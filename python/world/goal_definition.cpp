// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "goal_definition.hpp"
#include "modules/world/goal_definition/goal_definition.hpp"
#include "modules/geometry/polygon.hpp"

namespace py = pybind11;
using namespace modules::world::goal_definition;
using namespace modules::geometry;

void python_goal_definition(py::module m)
{
  py::class_<GoalDefinition>(m, "GoalDefinition")
      .def(py::init<const Polygon&>())
      .def("__repr__", [](const GoalDefinition &g) {
        return "bark.world.goal_definition.GoalDefinition";
      })
      .def(py::pickle(
        [](const GoalDefinition& g) -> py::tuple { // __getstate__
            /* Return a tuple that fully encodes the state of the object */
            return py::make_tuple(g.get_shape());
        },
        [](py::tuple t) { // __setstate__
          if (t.size() != 1)
                throw std::runtime_error("Invalid GoalDefinition state!");

          return GoalDefinition(t[0].cast<Polygon>());
                
        }));
}
