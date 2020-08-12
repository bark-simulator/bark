// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "preceding_agent_label_function.hpp"
#include "bark/world/observed_world.hpp"

bool bark::world::evaluation::PrecedingAgentLabelFunction::EvaluateAgent(
    const bark::world::ObservedWorld& observed_world,
    const bark::world::objects::AgentPtr& other_agent) const {
  bool is_in_front = false;
  auto agent_in_front = observed_world.GetAgentInFront();
  if (agent_in_front.first) {
    is_in_front =
        (agent_in_front.first->GetAgentId() == other_agent->GetAgentId());
  }
  return is_in_front;
}
