// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_GOAL_DEFINITION_POLYGON_HPP_
#define BARK_WORLD_GOAL_DEFINITION_POLYGON_HPP_

#include "bark/geometry/polygon.hpp"
#include "bark/world/goal_definition/goal_definition.hpp"

namespace bark {
namespace world {
namespace objects {
class Agent;
}
namespace goal_definition {

class GoalDefinitionPolygon : public GoalDefinition {
 public:
  GoalDefinitionPolygon() : GoalDefinition(), goal_shape_() {}
  explicit GoalDefinitionPolygon(const bark::geometry::Polygon& goal_shape)
      : goal_shape_(goal_shape) {}

  virtual bool AtGoal(const bark::world::objects::Agent& agent);

  const bark::geometry::Polygon& GetShape() const { return goal_shape_; }

 private:
  bark::geometry::Polygon goal_shape_;
};

}  // namespace goal_definition
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_GOAL_DEFINITION_POLYGON_HPP_
