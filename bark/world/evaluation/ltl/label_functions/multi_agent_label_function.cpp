// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "multi_agent_label_function.hpp"

#include "bark/world/observed_world.hpp"

using bark::world::evaluation::LabelMap;

LabelMap bark::world::evaluation::MultiAgentLabelFunction::Evaluate(
    const bark::world::ObservedWorld& observed_world) const {
  const auto other_agents = observed_world.GetValidOtherAgents();
  LabelMap labels;
  for (const auto& agent : other_agents) {
    bool res = this->EvaluateAgent(observed_world, agent.second);
    labels.insert({this->GetLabel(agent.first), res});
  }
  return labels;
}
