#ifndef ATG_ENGINE_SIM_INTAKE_H
#define ATG_ENGINE_SIM_INTAKE_H

#include "part.h"

#include "gas_system.h"

class Intake : public Part {
    public:
        enum class ForcedInductionType {
            None,
            Turbocharger,
            Supercharger
        };

        struct ForcedInductionParameters {
            ForcedInductionType type = ForcedInductionType::None;
            double maxBoostPressure = 0.0;
            double spoolUpTime = 0.25;
            double spoolDownTime = 0.35;
            double efficiency = 1.0;
            double idleBoostFraction = 0.0;
        };

        struct Parameters {
            // Plenum volume
            double volume;

            // Plenum dimensions
            double CrossSectionArea;

            // Input flow constant
            double InputFlowK;

            // Idle-circuit flow constant
            double IdleFlowK;

            // Flow rate from plenum to runner
            double RunnerFlowRate;

            // Molecular air fuel ratio (defaults to ideal for octane)
            double MolecularAfr = (25.0 / 2.0);

            // Throttle plate position at idle
            double IdleThrottlePlatePosition = 0.975;

            // Runner volume
            double RunnerLength = units::distance(4.0, units::inch);

            // Velocity decay factor
            double VelocityDecay = 0.5;

            // Forced induction configuration
            ForcedInductionParameters forcedInduction;
        };

    public:
        Intake();
        virtual ~Intake();

        void initialize(Parameters &params);
        virtual void destroy();

        void process(double dt);

        inline double getRunnerFlowRate() const { return m_runnerFlowRate; }
        inline double getThrottlePlatePosition() const { return m_idleThrottlePlatePosition * m_throttle; }
        inline double getRunnerLength() const { return m_runnerLength; }
        inline double getPlenumCrossSectionArea() const { return m_crossSectionArea; }
        inline double getVelocityDecay() const { return m_velocityDecay; }
        inline ForcedInductionType getForcedInductionType() const { return m_forcedInductionParameters.type; }
        inline double getBoostPressure() const { return m_boostPressure; }
        inline double getBoostLevel() const { return m_boostLevel; }
        inline double getBoostTemperature() const { return m_boostTemperature; }

        GasSystem m_system;
        double m_throttle;

        double m_flow;
        double m_flowRate;
        double m_totalFuelInjected;

    protected:
        void updateForcedInduction(double dt);

        double m_crossSectionArea;
        double m_inputFlowK;
        double m_idleFlowK;
        double m_runnerFlowRate;
        double m_molecularAfr;
        double m_idleThrottlePlatePosition;
        double m_runnerLength;
        double m_velocityDecay;

        ForcedInductionParameters m_forcedInductionParameters;
        double m_boostPressure;
        double m_boostLevel;
        double m_boostTemperature;

        GasSystem m_atmosphere;
};

#endif /* ATG_ENGINE_SIM_INTAKE_H */
