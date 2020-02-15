// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_WORLD_GOAL_DEFINITION_STATE_LIMITS_FRENET_HPP_
#define MODULES_WORLD_GOAL_DEFINITION_STATE_LIMITS_FRENET_HPP_

#include "modules/world/goal_definition/goal_definition.hpp"
#include "modules/geometry/polygon.hpp"

namespace modules {
namespace world {
namespace objects {
class Agent;
}
namespace goal_definition {


class GoalDefinitionStateLimitsFrenet : public GoalDefinition  {
 public:
  GoalDefinitionStateLimitsFrenet() :
                GoalDefinition(),
                center_line_(),
                max_lateral_distances_(),
                max_orientation_differences_(),
                velocity_range_() {}
  GoalDefinitionStateLimitsFrenet(
    const modules::geometry::Line& center_line,
    const std::pair<float,float> max_lateral_distances,
    const std::pair<float,float> max_orientation_differences,
    const std::pair<float, float> velocity_range);

  virtual bool AtGoal(const modules::world::objects::Agent& agent);

  const modules::geometry::Line& GetCenterLine() const {return center_line_;}
  const std::pair<float,float>& GetMaxLateralDistance() const {return max_lateral_distances_;}
  const std::pair<float,float>& GetMaxOrientationDifferences() const {return max_orientation_differences_;}
  const std::pair<float,float>& GetVelocityRange() const {return velocity_range_;}
  virtual const modules::geometry::Polygon& GetShape() const { return shape_;}

 private:
  const modules::geometry::Line center_line_;
  const std::pair<float,float> max_lateral_distances_;
  const std::pair<float,float> max_orientation_differences_;
  const std::pair<float,float> velocity_range_;
  modules::geometry::Polygon shape_;
};


}  // namespace goal_definition
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_GOAL_DEFINITION_STATE_LIMITS_FRENET_HPP_


