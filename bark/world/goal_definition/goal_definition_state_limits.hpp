// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_GOAL_DEFINITION_STATE_LIMITS_HPP_
#define BARK_WORLD_GOAL_DEFINITION_STATE_LIMITS_HPP_

#include "bark/geometry/polygon.hpp"
#include "bark/world/goal_definition/goal_definition.hpp"

namespace bark {
namespace world {
namespace objects {
class Agent;
}
namespace goal_definition {

class GoalDefinitionStateLimits : public GoalDefinition {
 public:
  GoalDefinitionStateLimits()
      : GoalDefinition(), xy_limits_(), angle_limits_() {}
  GoalDefinitionStateLimits(const bark::geometry::Polygon& xy_limits,
                            const std::pair<float, float> angle_limits)
      : xy_limits_(xy_limits), angle_limits_(angle_limits) {}

  virtual bool AtGoal(const bark::world::objects::Agent& agent);

  const bark::geometry::Polygon& GetXyLimits() const { return xy_limits_; }
  const bark::geometry::Polygon& GetShape() const { return xy_limits_; }
  const std::pair<float, float> GetAngleLimits() const { return angle_limits_; }

 private:
  bark::geometry::Polygon xy_limits_;
  std::pair<float, float> angle_limits_;
};

}  // namespace goal_definition
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_GOAL_DEFINITION_STATE_LIMITS_HPP_
