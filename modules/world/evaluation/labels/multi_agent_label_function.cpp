// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "multi_agent_label_function.hpp"

#include "modules/world/observed_world.hpp"

using modules::world::evaluation::LabelMap;

std::vector<LabelMap::value_type>
modules::world::evaluation::MultiAgentLabelFunction::Evaluate(
    const modules::world::ObservedWorld& observed_world) const {
  const auto other_agents = observed_world.GetValidOtherAgents();
  std::vector<LabelMap::value_type> labels;
  for (const auto& agent : other_agents) {
    bool res = this->EvaluateAgent(observed_world, agent.second);
    labels.emplace_back(this->GetLabel(agent.first), res);
  }
  return labels;
}
