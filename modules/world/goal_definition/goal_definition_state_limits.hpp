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


class GoalDefinitionStateLimits : public GoalDefinition  {
 public:
  GoalDefinitionStateLimits() :
                 GoalDefinition(),
                 xy_limits_(),
                 angle_limits_() {}
  GoalDefinitionStateLimits(const modules::geometry::Polygon& xy_limits,
                            const std::pair<float,float> angle_limits) :
                             xy_limits_(xy_limits),
                             angle_limits_(angle_limits) {}

  virtual bool AtGoal(const modules::world::objects::Agent& agent);

  const modules::geometry::Polygon& get_xy_limits() const {return xy_limits_;}
  const modules::geometry::Polygon& get_shape() const {return xy_limits_;}
  const std::pair<float,float> get_angle_limits() const {return angle_limits_;}

 private:
  modules::geometry::Polygon xy_limits_;
  std::pair<float,float> angle_limits_;
};


}  // namespace goal_definition
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_GOAL_DEFINITION_STATE_LIMITS_HPP_


