// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_GOAL_DEFINITION_STATE_LIMITS_FRENET_HPP_
#define BARK_WORLD_GOAL_DEFINITION_STATE_LIMITS_FRENET_HPP_

#include "bark/geometry/polygon.hpp"
#include "bark/world/goal_definition/goal_definition.hpp"

namespace bark {
namespace world {
namespace objects {
class Agent;
}
namespace goal_definition {

class GoalDefinitionStateLimitsFrenet : public GoalDefinition {
 public:
  GoalDefinitionStateLimitsFrenet()
      : GoalDefinition(),
        center_line_(),
        max_lateral_distances_(),
        max_orientation_differences_(),
        velocity_range_() {}
  GoalDefinitionStateLimitsFrenet(
      const bark::geometry::Line& center_line,
      const std::pair<float, float> max_lateral_distances,
      const std::pair<float, float> max_orientation_differences,
      const std::pair<float, float> velocity_range);

  virtual bool AtGoal(const bark::world::objects::Agent& agent);

  const bark::geometry::Line& GetCenterLine() const { return center_line_; }
  const std::pair<float, float>& GetMaxLateralDistance() const {
    return max_lateral_distances_;
  }
  const std::pair<float, float>& GetMaxOrientationDifferences() const {
    return max_orientation_differences_;
  }
  const std::pair<float, float>& GetVelocityRange() const {
    return velocity_range_;
  }
  virtual const bark::geometry::Polygon& GetShape() const { return shape_; }

 private:
  const bark::geometry::Line center_line_;
  const std::pair<float, float> max_lateral_distances_;
  const std::pair<float, float> max_orientation_differences_;
  const std::pair<float, float> velocity_range_;
  bark::geometry::Polygon shape_;
};

}  // namespace goal_definition
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_GOAL_DEFINITION_STATE_LIMITS_FRENET_HPP_
