// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef PYTHON_PYTHON_BINDINGS_WORLD_GOAL_DEFINITION_HPP_
#define PYTHON_PYTHON_BINDINGS_WORLD_GOAL_DEFINITION_HPP_

#include "python/common.hpp"
#include "modules/world/goal_definition/goal_definition.hpp"
#include "modules/world/objects/agent.hpp"
#include "modules/geometry/polygon.hpp"

namespace py = pybind11;
using modules::world::goal_definition::GoalDefinition;

class PyGoalDefinition : public GoalDefinition {
 public:
  using GoalDefinition::GoalDefinition;

  bool AtGoal(const modules::world::objects::Agent& agent) override {
    PYBIND11_OVERLOAD_PURE(
        bool,
        GoalDefinition,
        AtGoal,
        agent);
  }
  const modules::geometry::Polygon& get_shape() const override {
    PYBIND11_OVERLOAD(
        modules::geometry::Polygon&,
        GoalDefinition,
        get_shape);
  }

};


void python_goal_definition(py::module m);

#endif  // PYTHON_PYTHON_BINDINGS_WORLD_GOAL_DEFINITION_HPP_
