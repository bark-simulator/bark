// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include "agent.hpp"
#include "modules/world/objects/agent.hpp"
#include "modules/world/objects/object.hpp"
#include "modules/models/behavior/behavior_model.hpp"

namespace py = pybind11;
using namespace modules::world::objects;
using namespace modules::models::dynamic;
using namespace modules::commons;
using namespace modules::models::behavior;
using namespace modules::models::execution;
using namespace modules::geometry;

void python_agent(py::module m)
{
  py::class_<Agent, AgentPtr>(m, "Agent")
      .def(
          py::init<const State &,
          const BehaviorModelPtr &,
          const DynamicModelPtr &,
          const ExecutionModelPtr &,
          const Polygon &, Params *,
          const LaneId &,
          const MapInterfacePtr &,
          const Model3D &>(),
          py::arg("initial_state"),
          py::arg("behavior_model_ptr"),
          py::arg("dynamic_model_ptr"),
          py::arg("execution_model"),
          py::arg("shape"),
          py::arg("params"),
          py::arg("lane_id") = LaneId(0),
          py::arg("map_interface") = nullptr,
          py::arg("model_3d") = Model3D())
      .def("__repr__", [](const Agent &a) {
        return "bark.agent.Agent";
      })
      .def_property("route", &Agent::get_route_generator, &Agent::set_route_generator)
      .def_property_readonly("history", &Agent::get_state_input_history)
      .def_property_readonly("shape", &Agent::get_shape)
      .def_property_readonly("id", &Agent::get_agent_id)
      .def_property_readonly("followed_trajectory", &Agent::get_execution_trajectory)
      .def_property_readonly("planned_trajectory", &Agent::get_behavior_trajectory);

  py::class_<Object, ObjectPtr>(m, "Object")
      .def(
          py::init<const Polygon &,
          Params *,
          const Model3D &>(),
          py::arg("shape"),
          py::arg("params"),
          py::arg("model_3d") = Model3D())
      .def("__repr__", [](const Object &a) {
        return "bark.agent.Object";
      })
      .def_property_readonly("shape", &Object::get_shape)
      .def_property_readonly("id", &Object::get_agent_id);
}
