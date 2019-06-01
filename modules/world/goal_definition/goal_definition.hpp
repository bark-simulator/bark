// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_WORLD_OBJECTS_GOAL_DEFINITION_HPP_
#define MODULES_WORLD_OBJECTS_GOAL_DEFINITION_HPP_

#include "modules/geometry/polygon.hpp"

namespace modules {
namespace world {
namespace goal_definition {


class GoalDefinition  {
  public:
    GoalDefinition(const modules::geometry::Polygon& goal_shape) : goal_shape_(goal_shape) {}

    bool AtGoal(const modules::geometry::Polygon& agent_shape) const {
      return modules::geometry::Collide(agent_shape, goal_shape_);
    }

    const modules::geometry::Polygon& get_shape() const {return goal_shape_;}

  private:
    modules::geometry::Polygon goal_shape_;

};


}  // namespace objects
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_OBJECTS_GOAL_DEFINITION_HPP_

