// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/world/goal_definition/goal_definition_polygon.hpp"
#include "bark/world/objects/agent.hpp"

namespace bark {
namespace world {
namespace goal_definition {

bool GoalDefinitionPolygon::AtGoal(const bark::world::objects::Agent& agent) {
  return bark::geometry::Within(
      agent.GetPolygonFromState(agent.GetCurrentState()), goal_shape_);
}

}  // namespace goal_definition
}  // namespace world
}  // namespace bark