// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "evaluator_ltl.hpp"

#include <algorithm>
#include <string>
#include <vector>

#ifdef LTL_RULES
#include "ltl/rule_monitor.h"
#endif

#include "bark/world/observed_world.hpp"
#include "bark/world/world.hpp"

namespace bark {
namespace world {
namespace evaluation {

#ifdef LTL_RULES
EvaluatorLTL::EvaluatorLTL(bark::world::objects::AgentId agent_id,
                           const std::string& ltl_formula_str,
                           const LabelFunctions& label_functions)
    : safety_violations_(0),
      agent_id_(agent_id),
      ltl_formula_str_(ltl_formula_str),
      monitor_(RuleMonitor::MakeRule(ltl_formula_str, -1.0, 0)),
      label_functions_(label_functions) {
  if (!monitor_->IsAgentSpecific()) {
    rule_states_.emplace_back(monitor_->MakeRuleState()[0]);
  }
}

EvaluationReturn EvaluatorLTL::Evaluate(const bark::world::World& world) {
  auto cloned_world = world.Clone();
  if (world.GetAgent(agent_id_)) {
    return Evaluate(cloned_world->Observe({agent_id_})[0]);
  } else {
    return static_cast<int>(safety_violations_);
  }
}

EvaluationReturn EvaluatorLTL::Evaluate(
    const bark::world::ObservedWorld& observed_world) {
  // Only required if rule is in relation to other agents
  if (monitor_->IsAgentSpecific()) {
    auto other_agents = observed_world.GetValidOtherAgents();
    std::vector<int> current_agents;
    // Convert agent map to vector of ids
    std::transform(other_agents.begin(), other_agents.end(),
                   std::back_inserter(current_agents),
                   [](const AgentMap::value_type& e) {
                     return static_cast<int>(e.first);
                   });
    // Create rule states for new agents
    auto new_rule_states =
        monitor_->MakeRuleState(current_agents, GetKnownAgents());
    rule_states_.insert(rule_states_.end(), new_rule_states.begin(),
                        new_rule_states.end());
    // Add new agents to known agents
    AddKnownAgents(current_agents);
    // Remove rule states for non-existent agents
    RemoveRuleStates(current_agents);
  }

  // Obtain label values
  LabelMap labels = EvaluateLabels(observed_world);

  unsigned int guarantee_violations = 0;
  float penalty = 0.0f;
  // Evaluate for every rule state
  for (auto& rs : rule_states_) {
    // Check for violations of safety properties
    penalty = rs.GetAutomaton()->Evaluate(labels, rs);
    if (penalty != 0.0f) {
      safety_violations_++;
      VLOG(1) << "Rule \"" << ltl_formula_str_ << "\" violated in timestep "
              << observed_world.GetWorldTime() << " for agent ids "
              << rs.GetAgentIds() << " !";
    }
    // Check for violations of guarantee properties by assuming the trace
    // has ended.
    penalty = rs.GetAutomaton()->FinalTransit(rs);
    if (penalty != 0.0f) {
      // Do not make guarantee violations persistent
      // otherwise if more evaluations are performed, guarantee violations would
      // be counted twice!
      ++guarantee_violations;
    }
  }
  return static_cast<int>(safety_violations_ + guarantee_violations);
}
LabelMap EvaluatorLTL::EvaluateLabels(
    const ObservedWorld& observed_world) const {
  LabelMap labels;
  for (auto const& le : label_functions_) {
    auto label_map = le->Evaluate(observed_world);
    labels.insert(label_map.begin(), label_map.end());
  }
  return labels;
}

/// Return agent IDs for which rule states have been previously created
/// \return Agent IDs that were present in previous evaluations
std::vector<int> EvaluatorLTL::GetKnownAgents() {
  return std::vector<int>(known_agents_.begin(), known_agents_.end());
}

/// Add new agents to the list of known agents
/// \param new_agents New agents, not present in previous evaluations
void EvaluatorLTL::AddKnownAgents(const std::vector<int>& new_agents) {
  known_agents_.insert(new_agents.begin(), new_agents.end());
}

/// Remove all rule states, containing agents not present in the current world.
/// \param current_agents List of agent IDs in the current world
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

const std::vector<RuleState>& EvaluatorLTL::GetRuleStates() const {
  return rule_states_;
}

const LabelFunctions& EvaluatorLTL::GetLabelFunctions() const {
  return label_functions_;
}

#else
EvaluatorLTL::EvaluatorLTL(){};
#endif

}  // namespace evaluation
}  // namespace world
}  // namespace bark
