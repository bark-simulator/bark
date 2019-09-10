// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "goal_definition.hpp"
#include "modules/world/goal_definition/goal_definition_polygon.hpp"
#include "modules/geometry/polygon.hpp"

namespace py = pybind11;
using modules::world::goal_definition::GoalDefinition;
using modules::world::goal_definition::GoalDefinitionPtr;
using modules::world::goal_definition::GoalDefinitionPolygon;
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

          return GoalDefinitionPolygon(t[0].cast<Polygon>());
                
        }));
}
