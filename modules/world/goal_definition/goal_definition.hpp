// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_WORLD_GOAL_DEFINITION_HPP_
#define MODULES_WORLD_GOAL_DEFINITION_HPP_

#include <memory>

#include "modules/geometry/polygon.hpp"

namespace modules {
namespace world {
namespace objects {
  class Agent;
}
namespace goal_definition {


class GoalDefinition  {
  public:
    GoalDefinition() {}
    virtual ~GoalDefinition() {}

    virtual bool AtGoal(const modules::world::objects::Agent& agent) = 0;
    virtual const modules::geometry::Polygon& get_shape() const {}
};

typedef std::shared_ptr<GoalDefinition> GoalDefinitionPtr;

}  // namespace objects
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_GOAL_DEFINITION_HPP_

