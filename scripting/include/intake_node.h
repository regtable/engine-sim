#ifndef ATG_ENGINE_SIM_INTAKE_NODE_H
#define ATG_ENGINE_SIM_INTAKE_NODE_H

#include "object_reference_node.h"

#include "engine_context.h"
#include "function_node.h"

#include "engine_sim.h"

#include <algorithm>
#include <cctype>
#include <map>
#include <vector>

namespace es_script {

    class IntakeNode : public ObjectReferenceNode<IntakeNode> {
    public:
        IntakeNode() { /* void */ }
        virtual ~IntakeNode() { /* void */ }

        Intake *generate(EngineContext *context) {
            Intake *intake = context->getIntake(this);
            Intake::Parameters parameters = m_parameters;
            parameters.forcedInduction.type = parseForcedInductionType(m_forcedInductionTypeName);
            if (parameters.forcedInduction.maxBoostPressure < 0.0) {
                parameters.forcedInduction.maxBoostPressure = 0.0;
            }
            if (parameters.forcedInduction.efficiency <= 0.0) {
                parameters.forcedInduction.efficiency = 1.0;
            }
            intake->initialize(parameters);

            return intake;
        }

    protected:
        static Intake::ForcedInductionType parseForcedInductionType(const std::string &typeName) {
            std::string normalized;
            normalized.reserve(typeName.size());
            for (char c : typeName) {
                normalized.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
            }

            if (normalized == "turbo" || normalized == "turbocharger" || normalized == "turbo_charger") {
                return Intake::ForcedInductionType::Turbocharger;
            }
            else if (
                normalized == "super" || normalized == "supercharger"
                || normalized == "blower" || normalized == "super_charger")
            {
                return Intake::ForcedInductionType::Supercharger;
            }
            else {
                return Intake::ForcedInductionType::None;
            }
        }

        virtual void registerInputs() {
            addInput("plenum_volume", &m_parameters.volume);
            addInput("plenum_cross_section_area", &m_parameters.CrossSectionArea);
            addInput("intake_flow_rate", &m_parameters.InputFlowK);
            addInput("idle_flow_rate", &m_parameters.IdleFlowK);
            addInput("runner_flow_rate", &m_parameters.RunnerFlowRate);
            addInput("molecular_afr", &m_parameters.MolecularAfr);
            addInput("idle_throttle_plate_position", &m_parameters.IdleThrottlePlatePosition);
            addInput("throttle_gamma", &m_throttleGammaUnused);
            addInput("runner_length", &m_parameters.RunnerLength);
            addInput("velocity_decay", &m_parameters.VelocityDecay);
            addInput("forced_induction_type", &m_forcedInductionTypeName);
            addInput("forced_induction_max_boost", &m_parameters.forcedInduction.maxBoostPressure);
            addInput("forced_induction_spool_time", &m_parameters.forcedInduction.spoolUpTime);
            addInput("forced_induction_decay_time", &m_parameters.forcedInduction.spoolDownTime);
            addInput("forced_induction_efficiency", &m_parameters.forcedInduction.efficiency);
            addInput("forced_induction_idle_fraction", &m_parameters.forcedInduction.idleBoostFraction);

            ObjectReferenceNode<IntakeNode>::registerInputs();
        }

        virtual void _evaluate() {
            setOutput(this);

            // Read inputs
            readAllInputs();
        }

        double m_throttleGammaUnused = 0.0; // Deprecated; to be removed in a future release
        Intake::Parameters m_parameters;
        std::string m_forcedInductionTypeName = "none";
    };

} /* namespace es_script */

#endif /* ATG_ENGINE_SIM_INTAKE_NODE_H */
