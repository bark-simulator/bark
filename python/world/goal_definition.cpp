// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "goal_definition.hpp"
#include "modules/world/goal_definition/goal_definition_polygon.hpp"
#include "modules/world/goal_definition/goal_definition_state_limits.hpp"
#include "modules/world/goal_definition/goal_definition_state_limits_frenet.hpp"
#include "modules/world/goal_definition/goal_definition_sequential.hpp"
#include "modules/geometry/polygon.hpp"

namespace py = pybind11;
using modules::world::goal_definition::GoalDefinition;
using modules::world::goal_definition::GoalDefinitionPtr;
using modules::world::goal_definition::GoalDefinitionPolygon;
using modules::world::goal_definition::GoalDefinitionStateLimits;
using modules::world::goal_definition::GoalDefinitionSequential;
using modules::geometry::Polygon;
using modules::geometry::Line;

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
      .def_property_readonly("xy_limits",
        &GoalDefinitionStateLimits::get_xy_limits)
      .def_property_readonly("goal_shape",
        &GoalDefinitionStateLimits::get_xy_limits)
      .def_property_readonly("angle_limits",
        &GoalDefinitionStateLimits::get_angle_limits)
      .def(py::pickle(
        [](const GoalDefinitionStateLimits& g) -> py::tuple {  // __getstate__
            /* Return a tuple that fully encodes the state of the object */
            return py::make_tuple(g.get_xy_limits(), g.get_angle_limits());
        },
        [](py::tuple t) {  // __setstate__
          if (t.size() != 2)
                throw std::runtime_error("Invalid GoalDefinitionStateLimits state!");

          return new GoalDefinitionStateLimits(t[0].cast<Polygon>(), t[1].cast<
                  std::pair<float, float>>());
        }));

    py::class_<GoalDefinitionStateLimitsFrenet, GoalDefinition,
      std::shared_ptr<GoalDefinitionStateLimitsFrenet>>(m, "GoalDefinitionStateLimitsFrenet")
      .def(py::init<>())
      .def(py::init<const modules::geometry::Line&,
                            const std::pair<float,float>,
                            const std::pair<float,float>,
                            const std::pair<float, float>>())
      .def("__repr__", [](const GoalDefinitionStateLimitsFrenet &g) {
        return "bark.world.goal_definition.GoalDefinitionStateLimitsFrenet";
      })
      .def_property_readonly("center_line",
        &GoalDefinitionStateLimits::get_center_line)
      .def_property_readonly("max_lateral_distances",
        &GoalDefinitionStateLimits::get_max_lateral_distances)
      .def_property_readonly("max_orientation_differences",
        &GoalDefinitionStateLimits::get_max_orientation_differences)
      .def_property_readonly("velocity_range",
        &GoalDefinitionStateLimits::get_velocity_range)
      .def(py::pickle(
        [](const GoalDefinitionStateLimitsFrenet& g) -> py::tuple {  // __getstate__
            /* Return a tuple that fully encodes the state of the object */
            return py::make_tuple(g.get_center_line(),
                      get_max_lateral_distances(), get_max_orientation_differences(),
                      get_velocity_range());
        },
        [](py::tuple t) {  // __setstate__
          if (t.size() != 5)
                throw std::runtime_error("Invalid GoalDefinitionStateLimits state!");

          return new GoalDefinitionStateLimitsFrenet(t[0].cast<Line>(),
                  t[1].cast<std::pair<float, float>>(), t[2].cast<std::pair<float, float>>(),
                  t[3].cast<std::pair<float, float>>(), t[4].cast<std::pair<float, float>>());
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
