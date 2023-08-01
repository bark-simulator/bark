#ifndef EVALUATOR_ROBUSTNESS_STL_H_
#define EVALUATOR_ROBUSTNESS_STL_H_

#include "evaluator_ltl.hpp"

namespace bark {
namespace world {
namespace evaluation {

class RobustnessSTL : public EvaluatorLTL {
public:
    RobustnessSTL(bark::world::objects::AgentId agent_id,
                  const std::string& ltl_formula_str,
                  const LabelFunctions& label_functions);

    EvaluationReturn Evaluate(const bark::world::ObservedWorld& observed_world);

    float CalculateRobustness() const;

    float NormalizeRobustness(float robustness) const;

    unsigned int GetTotalRuleEvaluations() const;
    unsigned int GetRuleViolationCount() const;
    unsigned int GetRulePassCount() const;

private:
    unsigned int total_rule_evaluations_;
    unsigned int rule_violation_count_;
    unsigned int rule_pass_count_;
};

}  // namespace evaluation
}  // namespace world
}  // namespace bark

#endif  // EVALUATOR_ROBUSTNESS_STL_H_
