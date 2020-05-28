// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "direct_front_of_label_function.hpp"
#include "modules/world/observed_world.hpp"

bool modules::world::evaluation::DirectFrontOfLabelFunction::evaluate_agent(
    const modules::world::ObservedWorld& observed_world,
    const modules::world::objects::AgentPtr& other_agent) const {
  bool is_in_front = false;
  auto agent_in_front = observed_world.GetAgentInFront();
  if (agent_in_front.first) {
    is_in_front =
        (agent_in_front.first->GetAgentId() == other_agent->GetAgentId());
  }
  return is_in_front;
}
