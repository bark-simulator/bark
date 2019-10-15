// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_WORLD_GOAL_DEFINITION_POLYGON_HPP_
#define MODULES_WORLD_GOAL_DEFINITION_POLYGON_HPP_

#include "modules/world/goal_definition/goal_definition.hpp"
#include "modules/geometry/polygon.hpp"

namespace modules {
namespace world {
namespace objects {
class Agent;
}
namespace goal_definition {


class GoalDefinitionPolygon : public GoalDefinition  {
 public:
  GoalDefinitionPolygon() : GoalDefinition(), goal_shape_() {}
  explicit GoalDefinitionPolygon(const modules::geometry::Polygon& goal_shape) :
    goal_shape_(goal_shape) {}

  virtual bool AtGoal(const modules::world::objects::Agent& agent);

  const modules::geometry::Polygon& get_shape() const {return goal_shape_;}

 private:
  modules::geometry::Polygon goal_shape_;
};


}  // namespace goal_definition
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_GOAL_DEFINITION_POLYGON_HPP_


