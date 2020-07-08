// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "goal_definition.hpp"
#include <utility>
#include <vector>
#include "bark/geometry/polygon.hpp"
#include "bark/world/goal_definition/goal_definition_polygon.hpp"
#include "bark/world/goal_definition/goal_definition_sequential.hpp"
#include "bark/world/goal_definition/goal_definition_state_limits.hpp"
#include "bark/world/goal_definition/goal_definition_state_limits_frenet.hpp"

namespace py = pybind11;
using bark::geometry::Line;
using bark::geometry::Polygon;
using bark::world::goal_definition::GoalDefinition;
using bark::world::goal_definition::GoalDefinitionPolygon;
using bark::world::goal_definition::GoalDefinitionPtr;
using bark::world::goal_definition::GoalDefinitionSequential;
using bark::world::goal_definition::GoalDefinitionStateLimits;
using bark::world::goal_definition::GoalDefinitionStateLimitsFrenet;

void python_goal_definition(py::module m) {
  py::class_<GoalDefinition, PyGoalDefinition, GoalDefinitionPtr>(
      m, "GoalDefinition")
      .def(py::init<>())
      .def("AtGoal", &GoalDefinition::AtGoal);

  py::class_<GoalDefinitionPolygon, GoalDefinition,
             std::shared_ptr<GoalDefinitionPolygon>>(m, "GoalDefinitionPolygon")
      .def(py::init<>())
      .def(py::init<const Polygon&>())
      .def("__repr__",
           [](const GoalDefinitionPolygon& g) {
             return "bark.core.world.goal_definition.GoalDefinitionPolygon";
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
             std::shared_ptr<GoalDefinitionStateLimits>>(
      m, "GoalDefinitionStateLimits")
      .def(py::init<>())
      .def(py::init<const Polygon&, const std::pair<float, float>&>())
      .def("__repr__",
           [](const GoalDefinitionStateLimits& g) {
             return "bark.core.world.goal_definition.GoalDefinitionStateLimits";
           })
      .def_property_readonly("xy_limits",
                             &GoalDefinitionStateLimits::GetXyLimits)
      .def_property_readonly("goal_shape", &GoalDefinitionStateLimits::GetShape)
      .def_property_readonly("angle_limits",
                             &GoalDefinitionStateLimits::GetAngleLimits)
      .def(py::pickle(
          [](const GoalDefinitionStateLimits& g) -> py::tuple {
            return py::make_tuple(g.GetShape(), g.GetAngleLimits());
          },
          [](py::tuple t) {
            if (t.size() != 2)
              throw std::runtime_error(
                  "Invalid GoalDefinitionStateLimits state!");  // NOLINT
            return new GoalDefinitionStateLimits(
                t[0].cast<Polygon>(), t[1].cast<std::pair<float, float>>());
          }));

  py::class_<GoalDefinitionStateLimitsFrenet, GoalDefinition,
             std::shared_ptr<GoalDefinitionStateLimitsFrenet>>(
      m, "GoalDefinitionStateLimitsFrenet")
      .def(py::init<>())
      .def(py::init<const bark::geometry::Line&, const std::pair<float, float>,
                    const std::pair<float, float>,
                    const std::pair<float, float>>())
      .def("__repr__",
           [](const GoalDefinitionStateLimitsFrenet& g) {
             return "bark.core.world.goal_definition."
                    "GoalDefinitionStateLimitsFrenet";
           })
      .def_property_readonly("center_line",
                             &GoalDefinitionStateLimitsFrenet::GetCenterLine)
      .def_property_readonly(
          "max_lateral_distances",
          &GoalDefinitionStateLimitsFrenet::GetMaxLateralDistance)
      .def_property_readonly(
          "max_orientation_differences",
          &GoalDefinitionStateLimitsFrenet::GetMaxOrientationDifferences)
      .def_property_readonly("velocity_range",
                             &GoalDefinitionStateLimitsFrenet::GetVelocityRange)
      .def_property_readonly("goal_shape",
                             &GoalDefinitionStateLimitsFrenet::GetShape)
      .def(py::pickle(
          [](const GoalDefinitionStateLimitsFrenet& g)
              -> py::tuple {  // __getstate__
            /* Return a tuple that fully encodes the state of the object */
            return py::make_tuple(g.GetCenterLine(), g.GetMaxLateralDistance(),
                                  g.GetMaxOrientationDifferences(),
                                  g.GetVelocityRange());
          },
          [](py::tuple t) {  // __setstate__
            if (t.size() != 4)
              throw std::runtime_error(
                  "Invalid GoalDefinitionStateLimitsFrenet state!");

            return new GoalDefinitionStateLimitsFrenet(
                t[0].cast<Line>(), t[1].cast<std::pair<float, float>>(),
                t[2].cast<std::pair<float, float>>(),
                t[3].cast<std::pair<float, float>>());
          }));

  py::class_<GoalDefinitionSequential, GoalDefinition,
             std::shared_ptr<GoalDefinitionSequential>>(
      m, "GoalDefinitionSequential")
      .def(py::init<>())
      .def(py::init<const std::vector<GoalDefinitionPtr>&>())
      .def("__repr__",
           [](const GoalDefinitionSequential& g) {
             return "bark.core.world.goal_definition.GoalDefinitionSequential";
           })
      .def("GetNextGoal", &GoalDefinitionSequential::GetNextGoal)
      .def("GetCurrentGoal", &GoalDefinitionSequential::GetCurrentGoal)
      .def_property_readonly("goal_shape", &GoalDefinitionSequential::GetShape)
      .def_property_readonly("sequential_goals",
                             &GoalDefinitionSequential::GetSequentialGoals)
      .def(py::pickle(
          [](const GoalDefinitionSequential& g) -> py::tuple {  // __getstate__
            /* Return a tuple that fully encodes the state of the object */
            return py::make_tuple(g.GetSequentialGoals());
          },
          [](py::tuple t) {
            if (t.size() != 1)
              throw std::runtime_error(
                  "Invalid GoalDefinitionSequential state!");
            return new GoalDefinitionSequential(
                t[0].cast<std::vector<GoalDefinitionPtr>>());
          }));
}
