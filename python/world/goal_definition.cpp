// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include <utility>
#include <vector>
#include "goal_definition.hpp"
#include "modules/world/goal_definition/goal_definition_polygon.hpp"
#include "modules/world/goal_definition/goal_definition_state_limits.hpp"
#include "modules/world/goal_definition/goal_definition_sequential.hpp"
#include "modules/geometry/polygon.hpp"

namespace py = pybind11;
using modules::world::goal_definition::GoalDefinition;
using modules::world::goal_definition::GoalDefinitionPtr;
using modules::world::goal_definition::GoalDefinitionPolygon;
using modules::world::goal_definition::GoalDefinitionStateLimits;
using modules::world::goal_definition::GoalDefinitionSequential;
using modules::geometry::Polygon;

void python_goal_definition(py::module m) {
  py::class_<GoalDefinition,
             PyGoalDefinition,
             GoalDefinitionPtr>(m, "GoalDefinition")
    .def(py::init<>())
    .def("AtGoal", &GoalDefinition::AtGoal);

  py::class_<GoalDefinitionPolygon, GoalDefinition,
          std::shared_ptr<GoalDefinitionPolygon>>(m, "GoalDefinitionPolygon")
    .def(py::init<>())
    .def(py::init<const Polygon&>())
    .def("__repr__", [](const GoalDefinitionPolygon &g) {
      return "bark.world.goal_definition.GoalDefinitionPolygon";
    })
    .def_property_readonly("goal_shape", &GoalDefinitionPolygon::GetShape)
    .def(py::pickle(
      [](const GoalDefinitionPolygon& g) -> py::tuple {
          return py::make_tuple(g.GetShape());
      },
      [](py::tuple t) {
        if (t.size() != 1)
              throw std::runtime_error("Invalid GoalDefinitionPolygon state!");

        return new GoalDefinitionPolygon(t[0].cast<Polygon>());
      }));

    py::class_<GoalDefinitionStateLimits, GoalDefinition,
      std::shared_ptr<GoalDefinitionStateLimits>>(m,
        "GoalDefinitionStateLimits")
      .def(py::init<>())
      .def(py::init<const Polygon&, const std::pair<float, float>&>())
      .def("__repr__", [](const GoalDefinitionStateLimits &g) {
        return "bark.world.goal_definition.GoalDefinitionStateLimits";
      })
      .def_property_readonly("xy_limits",
        &GoalDefinitionStateLimits::GetXyLimits)
      .def_property_readonly("goal_shape",
        &GoalDefinitionStateLimits::GetShape)
      .def_property_readonly("angle_limits",
        &GoalDefinitionStateLimits::GetAngleLimits)
      .def(py::pickle(
        [](const GoalDefinitionStateLimits& g) -> py::tuple {
            return py::make_tuple(g.GetShape(), g.GetAngleLimits());
        },
        [](py::tuple t) {
          if (t.size() != 2)
            throw std::runtime_error("Invalid GoalDefinitionStateLimits state!");  // NOLINT
          return new GoalDefinitionStateLimits(t[0].cast<Polygon>(), t[1].cast<
                  std::pair<float, float>>());
        }));

    py::class_<GoalDefinitionSequential, GoalDefinition,
        std::shared_ptr<GoalDefinitionSequential>>(m, "GoalDefinitionSequential")  // NOLINT
      .def(py::init<>())
      .def(py::init<const std::vector<GoalDefinitionPtr>&>())
      .def("__repr__", [](const GoalDefinitionSequential &g) {
        return "bark.world.goal_definition.GoalDefinitionSequential";
      })
      .def("GetNextGoal", &GoalDefinitionSequential::GetNextGoal)
      .def("GetCurrentGoal", &GoalDefinitionSequential::GetCurrentGoal)
      .def_property_readonly("goal_shape", &GoalDefinitionSequential::GetShape)
      .def_property_readonly("sequential_goals",
        &GoalDefinitionSequential::GetSequentialGoals)
      .def(py::pickle(
        [](const GoalDefinitionSequential& g) -> py::tuple {
          return py::make_tuple(g.GetSequentialGoals());
        },
        [](py::tuple t) {
          if (t.size() != 1)
            throw std::runtime_error("Invalid GoalDefinitionSequential state!");
          return new GoalDefinitionSequential(
            t[0].cast<std::vector<GoalDefinitionPtr>>());
        }));




}
