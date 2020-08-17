// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_GOAL_DEFINITION_HPP_
#define BARK_WORLD_GOAL_DEFINITION_HPP_

#include <memory>

#include "bark/geometry/polygon.hpp"

namespace bark {
namespace world {
namespace objects {
class Agent;
}
namespace goal_definition {

class GoalDefinition {
 public:
  GoalDefinition() {}
  virtual ~GoalDefinition() {}

  virtual bool AtGoal(const bark::world::objects::Agent& agent) = 0;

  virtual const bark::geometry::Polygon& GetShape() const {}
};

typedef std::shared_ptr<GoalDefinition> GoalDefinitionPtr;

}  // namespace goal_definition
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_GOAL_DEFINITION_HPP_
