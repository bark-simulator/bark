// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "evaluator_ltl.hpp"

#include "ltl/rule_monitor.h"
#include "modules/world/observed_world.hpp"
#include "modules/world/world.hpp"

namespace modules {
namespace world {
namespace evaluation {

EvaluatorLTL::EvaluatorLTL(
    modules::world::objects::AgentId agent_id,
    const std::string& ltl_formula_str)
    : violation_count_(0),
      agent_id_(agent_id),
      ltl_formula_str_(ltl_formula_str),
      monitor_(RuleMonitor::MakeRule(ltl_formula_str, -1.0, 0)) {
  if (!monitor_->IsAgentSpecific()) {
    rule_states_.emplace_back(monitor_->MakeRuleState()[0]);
  }
}

EvaluationReturn EvaluatorLTL::Evaluate(const modules::world::World& world) {
  return Evaluate(world.Observe({agent_id_})[0]);
}

EvaluationReturn EvaluatorLTL::Evaluate(
    const modules::world::ObservedWorld& observed_world) {
  if (monitor_->IsAgentSpecific()) {
    auto current_agents = AgentListToIds(observed_world.GetOtherAgents());
    auto new_rule_states =
        monitor_->MakeRuleState(current_agents, GetKnownAgents());
    rule_states_.insert(rule_states_.end(), new_rule_states.begin(),
                        new_rule_states.end());
    AddKnownAgents(current_agents);
    RemoveRuleStates(current_agents);
  }
  auto labels = observed_world.EvaluateLabels();
  unsigned int final_violation = 0;
  for (auto& rs : rule_states_) {
    unsigned int old_violation_count = rs.GetViolationCount();
    rs.GetAutomaton()->Evaluate(labels, rs);
    if (rs.GetViolationCount() > old_violation_count) {
      violation_count_++;
    }
    float penalty = rs.GetAutomaton()->FinalTransit(rs);
    // Do not count final violations to the overall violation count
    // => We only assume this is the last transition
    if (penalty != 0.0) {
      ++final_violation;
    }
  }
  return static_cast<int>(violation_count_ + final_violation);
}

std::vector<int> EvaluatorLTL::GetKnownAgents() {
  return std::vector<int>(known_agents_.begin(), known_agents_.end());
}

void EvaluatorLTL::AddKnownAgents(const std::vector<int>& new_agents) {
  known_agents_.insert(new_agents.begin(), new_agents.end());
}

void EvaluatorLTL::RemoveRuleStates(std::vector<int> current_agents) {
  std::sort(current_agents.begin(), current_agents.end());
  auto iter = rule_states_.begin();
  std::vector<int> diff;
  while (iter != rule_states_.end()) {
    diff.clear();
    auto agent_ids = iter->GetAgentIds();
    std::sort(agent_ids.begin(), agent_ids.end());
    // Find agents referenced by rule state but not existent in world
    std::set_difference(agent_ids.begin(), agent_ids.end(),
                        current_agents.begin(), current_agents.end(),
                        std::back_inserter(diff));
    if (!diff.empty()) {
      iter = rule_states_.erase(iter);
    } else {
      ++iter;
    }
  }
}

std::vector<int> EvaluatorLTL::AgentListToIds(const AgentMap& agent_map) {
  std::vector<int> agent_ids;
  std::transform(
      agent_map.begin(), agent_map.end(), std::back_inserter(agent_ids),
      [](const AgentMap::value_type& e) { return static_cast<int>(e.first); });
  return agent_ids;
}

const std::vector<RuleState>& EvaluatorLTL::GetRuleStates() const {
  return rule_states_;
}

}  // namespace evaluation
}  // namespace world
}  // namespace modules
