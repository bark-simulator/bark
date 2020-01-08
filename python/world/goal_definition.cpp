// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

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

void python_goal_definition(py::module m)
{
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
      .def_property_readonly("goal_shape", &GoalDefinitionPolygon::get_shape)
      .def(py::pickle(
        [](const GoalDefinitionPolygon& g) -> py::tuple { // __getstate__
            /* Return a tuple that fully encodes the state of the object */
            return py::make_tuple(g.get_shape());
        },
        [](py::tuple t) { // __setstate__
          if (t.size() != 1)
                throw std::runtime_error("Invalid GoalDefinitionPolygon state!");

          return new GoalDefinitionPolygon(t[0].cast<Polygon>());
                
        }));

    py::class_<GoalDefinitionStateLimits, GoalDefinition,
          std::shared_ptr<GoalDefinitionStateLimits>>(m, "GoalDefinitionStateLimits")
      .def(py::init<>())
      .def(py::init<const Polygon&, const std::pair<float, float>&>())
      .def("__repr__", [](const GoalDefinitionStateLimits &g) {
        return "bark.world.goal_definition.GoalDefinitionStateLimits";
      })
      .def_property_readonly("xy_limits", &GoalDefinitionStateLimits::get_xy_limits)
      .def_property_readonly("angle_limits", &GoalDefinitionStateLimits::get_angle_limits)
      .def(py::pickle(
        [](const GoalDefinitionStateLimits& g) -> py::tuple { // __getstate__
            /* Return a tuple that fully encodes the state of the object */
            return py::make_tuple(g.get_xy_limits(), g.get_angle_limits());
        },
        [](py::tuple t) { // __setstate__
          if (t.size() != 2)
                throw std::runtime_error("Invalid GoalDefinitionStateLimits state!");

          return new GoalDefinitionStateLimits(t[0].cast<Polygon>(), t[1].cast<
                  std::pair<float, float>>());
        }));

      
      py::class_<GoalDefinitionSequential, GoalDefinition,
        std::shared_ptr<GoalDefinitionSequential>>(m, "GoalDefinitionSequential")
    .def(py::init<>())
    .def(py::init<const std::vector<GoalDefinitionPtr>&>())
    .def("__repr__", [](const GoalDefinitionSequential &g) {
      return "bark.world.goal_definition.GoalDefinitionSequential";
    })
    .def("GetNextGoal", &GoalDefinitionSequential::GetNextGoal)
    .def("GetCurrentGoal", &GoalDefinitionSequential::GetCurrentGoal)
    .def_property_readonly("goal_shape", &GoalDefinitionSequential::get_shape)
    .def_property_readonly("sequential_goals", &GoalDefinitionSequential::get_sequential_goals)
    .def(py::pickle(
        [](const GoalDefinitionSequential& g) -> py::tuple { // __getstate__
            /* Return a tuple that fully encodes the state of the object */
            return py::make_tuple(g.get_sequential_goals());
        },
        [](py::tuple t) { // __setstate__
          if (t.size() != 1)
                throw std::runtime_error("Invalid GoalDefinitionSequential state!");

          return new GoalDefinitionSequential(t[0].cast<std::vector<GoalDefinitionPtr>>());
        }));




}
