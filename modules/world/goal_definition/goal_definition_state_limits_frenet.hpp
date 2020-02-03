// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_WORLD_GOAL_DEFINITION_STATE_LIMITS_HPP_
#define MODULES_WORLD_GOAL_DEFINITION_STATE_LIMITS_HPP_

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
  GoalDefinitionStateLimits() :
                 GoalDefinition(),
                 xy_limits_(),
                 angle_limits_() {}
  GoalDefinitionStateLimits(const modules::geometry::Line& center_line,
                            const std::pair<float,float> max_lateral_distances,
                            const std::pair<float,float> max_orientation_differences,
                            const std::pair<float, float> velocity_range) :
                            center_line_(center_line),
                            max_lateral_distances_(max_lateral_distances),
                            max_orientation_differences_(max_orientation_differences),
                            velocity_range_(velocity_range);

  virtual bool AtGoal(const modules::world::objects::Agent& agent);

  const modules::geometry::Polygon& get_center_line() const {return center_line_;}
  const modules::geometry::Polygon& get_max_lateral_distances() const {return max_lateral_distances_;}
  const modules::geometry::Polygon& get_max_orientation_differences() const {return max_orientation_differences_;}
  const modules::geometry::Polygon& get_velocity_range() const {return velocity_range_;}
  const modules::geometry::Polygon& get_shape() const { return shape_;}

 private:
  const modules::geometry::Line center_line_;
  const std::pair<float,float> max_lateral_distances_;
  const std::pair<float,float> max_orientation_differences_;
  const std::pair<float,float> velocity_range_;
  const modules::geometry::Polygon shape_;
};


}  // namespace goal_definition
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_GOAL_DEFINITION_STATE_LIMITS_HPP_


