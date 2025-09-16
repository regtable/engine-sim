#include "../include/intake.h"

#include "../include/units.h"

#include <algorithm>
#include <cmath>

Intake::Intake() {
    m_inputFlowK = 0;
    m_idleFlowK = 0;
    m_flow = 0;
    m_throttle = 1.0;
    m_idleThrottlePlatePosition = 0.0;
    m_crossSectionArea = 0.0;
    m_flowRate = 0;
    m_totalFuelInjected = 0;
    m_molecularAfr = 0;
    m_runnerLength = 0;
    m_boostPressure = 0.0;
    m_boostLevel = 0.0;
    m_boostTemperature = units::celcius(25.0);
}

Intake::~Intake() {
    /* void */
}

void Intake::initialize(Parameters &params) {
    const double width = std::sqrt(params.CrossSectionArea);
    m_system.initialize(
        units::pressure(1.0, units::atm),
        params.volume,
        units::celcius(25.0));
    m_system.setGeometry(
        width,
        params.volume / params.CrossSectionArea,
        1.0,
        0.0);

    m_atmosphere.initialize(
        units::pressure(1.0, units::atm),
        units::volume(1000.0, units::m3),
        units::celcius(25.0));
    m_atmosphere.setGeometry(
        units::distance(100.0, units::m),
        units::distance(100.0, units::m),
        1.0,
        0.0);

    m_inputFlowK = params.InputFlowK;
    m_molecularAfr = params.MolecularAfr;
    m_idleFlowK = params.IdleFlowK;
    m_idleThrottlePlatePosition = params.IdleThrottlePlatePosition;
    m_runnerLength = params.RunnerLength;
    m_crossSectionArea = params.CrossSectionArea;
    m_velocityDecay = params.VelocityDecay;
    m_runnerFlowRate = params.RunnerFlowRate;
    m_forcedInductionParameters = params.forcedInduction;
    m_boostLevel = std::clamp(m_forcedInductionParameters.idleBoostFraction, 0.0, 1.0);
    m_boostPressure = m_boostLevel * std::max(0.0, m_forcedInductionParameters.maxBoostPressure);
    m_boostTemperature = units::celcius(25.0);
}

void Intake::destroy() {
    /* void */
}

void Intake::process(double dt) {
    updateForcedInduction(dt);

    const double ideal_afr = 0.8 * m_molecularAfr * 4;
    const double current_afr = (m_system.mix().p_o2 + m_system.mix().p_inert) / m_system.mix().p_fuel;

    const double p_air = ideal_afr / (1 + ideal_afr);
    GasSystem::Mix fuelAirMix;
    fuelAirMix.p_fuel = 1 - p_air;
    fuelAirMix.p_inert = p_air * 0.75;
    fuelAirMix.p_o2 = p_air * 0.25;

    const double idle_afr = 2.0;
    const double p_idle_air = idle_afr / (1 + idle_afr);
    GasSystem::Mix fuelMix;
    fuelMix.p_fuel = (1.0 - p_idle_air);
    fuelMix.p_inert = p_idle_air * 0.75;
    fuelMix.p_o2 = p_idle_air * 0.25;

    const double throttle = getThrottlePlatePosition();
    const double flowAttenuation = std::cos(throttle * constants::pi / 2);

    const double ambientPressure = units::pressure(1.0, units::atm);
    const double ambientTemperature = units::celcius(25.0);
    const double sourcePressure = ambientPressure + m_boostPressure;
    const double efficiency = std::max(0.05, m_forcedInductionParameters.efficiency);
    const double hcr = m_system.heatCapacityRatio();
    double sourceTemperature = ambientTemperature;

    if (m_boostPressure > 0.0) {
        const double pressureRatio = sourcePressure / ambientPressure;
        const double temperatureRise = std::pow(pressureRatio, (hcr - 1.0) / hcr) - 1.0;
        sourceTemperature = ambientTemperature * (1.0 + temperatureRise / efficiency);
    }

    m_boostTemperature = sourceTemperature;

    GasSystem::FlowParameters flowParams;
    flowParams.crossSectionArea_0 = units::area(10, units::m2);
    flowParams.crossSectionArea_1 = m_crossSectionArea;
    flowParams.direction_x = 0.0;
    flowParams.direction_y = -1.0;
    flowParams.dt = dt;

    m_atmosphere.reset(sourcePressure, sourceTemperature, fuelAirMix);
    flowParams.system_0 = &m_atmosphere;
    flowParams.system_1 = &m_system;
    flowParams.k_flow = flowAttenuation * m_inputFlowK;
    m_flow = m_system.flow(flowParams);

    m_atmosphere.reset(sourcePressure, sourceTemperature, fuelMix);
    flowParams.system_0 = &m_atmosphere;
    flowParams.system_1 = &m_system;
    flowParams.k_flow = m_idleFlowK;
    const double idleCircuitFlow = m_system.flow(flowParams);

    m_system.dissipateExcessVelocity();
    m_system.updateVelocity(dt, m_velocityDecay);

    if (m_flow > 0) {
        m_totalFuelInjected += fuelAirMix.p_fuel * m_flow;
    }

    if (idleCircuitFlow > 0) {
        m_totalFuelInjected += fuelMix.p_fuel * idleCircuitFlow;
    }
}

void Intake::updateForcedInduction(double dt) {
    if (m_forcedInductionParameters.type == ForcedInductionType::None
        || m_forcedInductionParameters.maxBoostPressure <= 0.0)
    {
        m_boostLevel = std::clamp(m_forcedInductionParameters.idleBoostFraction, 0.0, 1.0);
        m_boostPressure = 0.0;
        return;
    }

    const double idleLevel = std::clamp(m_forcedInductionParameters.idleBoostFraction, 0.0, 1.0);
    double target = std::clamp(m_throttle, 0.0, 1.0);
    target = std::max(target, idleLevel);

    const double spoolUpTime = std::max(m_forcedInductionParameters.spoolUpTime, 1e-3);
    const double spoolDownTime = std::max(m_forcedInductionParameters.spoolDownTime, 1e-3);

    double newLevel = m_boostLevel;
    if (target > newLevel) {
        const double delta = dt / spoolUpTime;
        newLevel = std::min(target, newLevel + delta);
    }
    else {
        const double delta = dt / spoolDownTime;
        newLevel = std::max(target, newLevel - delta);
    }

    if (m_forcedInductionParameters.type == ForcedInductionType::Supercharger) {
        // Superchargers respond almost instantly; bias towards the target level.
        const double snap = dt / std::max(1e-3, 0.5 * spoolUpTime);
        newLevel = std::min(target, newLevel + snap * (target - newLevel));
    }

    m_boostLevel = std::clamp(newLevel, idleLevel, 1.0);
    m_boostPressure = m_boostLevel * std::max(0.0, m_forcedInductionParameters.maxBoostPressure);
}
