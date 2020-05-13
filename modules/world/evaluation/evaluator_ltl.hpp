// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_EVALUATION_EVALUATOR_LTL_HPP_
#define MODULES_WORLD_EVALUATION_EVALUATOR_LTL_HPP_

#include "ltl/rule_state.h"
#include "modules/world/evaluation/base_evaluator.hpp"
#include "modules/world/evaluation/labels/base_label_evaluator.hpp"
#include "modules/world/objects/agent.hpp"
#include "modules/world/world.hpp"

namespace modules {
namespace world {
namespace evaluation {

using ltl::RuleState;
using ltl::RuleMonitor;
using objects::AgentId;

class EvaluatorLTL : public BaseEvaluator {
 public:
  EvaluatorLTL(modules::world::objects::AgentId agent_id,
               const std::string& ltl_formula_str);
  EvaluationReturn Evaluate(const world::World& world) override;
  const std::vector<RuleState>& GetRuleStates() const;

 private:
  EvaluationReturn Evaluate(
      const world::ObservedWorld& observed_world) override;
  static std::vector<int> AgentListToIds(const AgentMap& agent_map);

  std::vector<int> GetKnownAgents();
  void AddKnownAgents(const std::vector<int>& new_agents);
  void RemoveRuleStates(std::vector<int> current_agents);

  unsigned int violation_count_;
  AgentId agent_id_;
  std::string ltl_formula_str_;
  std::vector<RuleState> rule_states_;
  RuleMonitor::RuleMonitorSPtr monitor_;
  std::set<AgentId> known_agents_;
};
}  // namespace evaluation
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_EVALUATION_EVALUATOR_LTL_HPP_
