// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef PYTHON_PYTHON_BINDINGS_WORLD_GOAL_DEFINITION_HPP_
#define PYTHON_PYTHON_BINDINGS_WORLD_GOAL_DEFINITION_HPP_

#include "bark/geometry/polygon.hpp"
#include "bark/python_wrapper/common.hpp"
#include "bark/world/goal_definition/goal_definition.hpp"
#include "bark/world/objects/agent.hpp"

namespace py = pybind11;
using bark::world::goal_definition::GoalDefinition;

class PyGoalDefinition : public GoalDefinition {
 public:
  using GoalDefinition::GoalDefinition;

  bool AtGoal(const bark::world::objects::Agent& agent) override {
    PYBIND11_OVERLOAD_PURE(bool, GoalDefinition, AtGoal, agent);
  }
  const bark::geometry::Polygon& GetShape() const override {
    PYBIND11_OVERLOAD(bark::geometry::Polygon&, GoalDefinition, GetShape);
  }
};

void python_goal_definition(py::module m);

#endif  // PYTHON_PYTHON_BINDINGS_WORLD_GOAL_DEFINITION_HPP_
