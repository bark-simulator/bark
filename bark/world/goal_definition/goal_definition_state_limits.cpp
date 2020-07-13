// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/world/goal_definition/goal_definition_state_limits.hpp"
#include "bark/world/objects/agent.hpp"

namespace bark {
namespace world {
namespace goal_definition {

bool GoalDefinitionStateLimits::AtGoal(
    const bark::world::objects::Agent& agent) {
  const auto agent_state = agent.GetCurrentState();
  auto agent_angle = bark::geometry::Norm0To2PI(
      agent_state[bark::models::dynamic::StateDefinition::THETA_POSITION]);
  const bark::geometry::Point2d agent_pos = agent.GetCurrentPosition();
  return (bark::geometry::Within(agent_pos, xy_limits_) &&
          agent_angle >= angle_limits_.first &&
          agent_angle <= angle_limits_.second);
}

}  // namespace goal_definition
}  // namespace world
}  // namespace bark