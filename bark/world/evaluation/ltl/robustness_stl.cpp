#include "evaluator_ltl.hpp"
#include "robustness_stl.hpp"

#include <vector>
#include <map>
#include <cmath>

namespace bark{
namespace world{
namespace evaluation {


class RobustnessSTL : public EvaluatorLTL {
public:
    RobustnessSTL(bark::world::objects::AgentId agent_id,
                  const std::string& ltl_formula_str,
                  const LabelFunctions& label_functions)
          : EvaluatorLTL(agent_id, ltl_formula_str, label_functions),
            total_rule_evaluations_(0),
            rule_violation_count_(0),
            rule_pass_count_(0){}

    EvaluationReturn Evaluate(const bark::world::ObservedWorld& observed_world) {
        ++total_rule_evaluations_;
        EvaluationReturn result = EvaluatorLTL::Evaluate(observed_world);
        if (result == 0) {  // assuming that 0 means no violations
            ++rule_pass_count_;
        } else {
            ++rule_violation_count_;
        }
        return result;
    }

    float CalculateRobustness() const {
        const float EPS = 1e-9;
        return rule_pass_count_ / (static_cast<float>(total_rule_evaluations_) + EPS);
    }

    float NormalizeRobustness(float robustness) const {
        float pos_min = 0.0;
        float pos_max = 1.0;
        float neg_min = 0.0;
        float neg_max = 1.0;

        if (robustness > 0) {
            return (robustness - pos_min) / (pos_max_ - pos_min);
        } else {
            return -1 * (robustness - neg_min) / (neg_max_ - neg_min);
        }
    }




    unsigned int GetTotalRuleEvaluations() const { return total_rule_evaluations_; }
    unsigned int GetRuleViolationCount() const { return rule_violation_count_; }
    unsigned int GetRulePassCount() const { return rule_pass_count_; }

private:
    unsigned int total_rule_evaluations_;
    unsigned int rule_violation_count_;
    unsigned int rule_pass_count_;
};


}  // namespace evaluation
}  // namespace world
}  // namespace bark