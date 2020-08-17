// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_EVALUATION_LTL_EVALUATOR_LTL_HPP_
#define BARK_WORLD_EVALUATION_LTL_EVALUATOR_LTL_HPP_

#include <set>
#include <string>
#include <vector>

#include "bark/world/evaluation/base_evaluator.hpp"
#include "bark/world/objects/agent.hpp"
#include "bark/world/world.hpp"
#ifdef LTL_RULES
#include "ltl/rule_state.h"
#endif
#include "bark/world/evaluation/ltl/label_functions/base_label_function.hpp"

namespace bark {
namespace world {
namespace evaluation {

#ifdef LTL_RULES
using ltl::RuleMonitor;
using ltl::RuleState;
#endif
typedef std::vector<LabelFunctionPtr> LabelFunctions;

using objects::AgentId;

class EvaluatorLTL : public BaseEvaluator {
 public:
#ifdef LTL_RULES
  EvaluatorLTL(bark::world::objects::AgentId agent_id,
               const std::string& ltl_formula_str,
               const LabelFunctions& label_functions);
  EvaluationReturn Evaluate(const world::World& world) override;
  const std::vector<RuleState>& GetRuleStates() const;
  const LabelFunctions& GetLabelFunctions() const;

 private:
  EvaluationReturn Evaluate(
      const world::ObservedWorld& observed_world) override;
  std::vector<int> GetKnownAgents();
  void AddKnownAgents(const std::vector<int>& new_agents);
  void RemoveRuleStates(std::vector<int> current_agents);

  unsigned int safety_violations_;
  AgentId agent_id_;
  std::string ltl_formula_str_;
  std::vector<RuleState> rule_states_;
  RuleMonitor::RuleMonitorSPtr monitor_;
  std::set<AgentId> known_agents_;
  LabelFunctions label_functions_;
  LabelMap EvaluateLabels(const ObservedWorld& observed_world) const;
#else
  EvaluatorLTL();
#endif
};
}  // namespace evaluation
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_EVALUATION_LTL_EVALUATOR_LTL_HPP_
