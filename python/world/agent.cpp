// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include "agent.hpp"
#include "modules/world/objects/agent.hpp"
#include "modules/world/objects/object.hpp"
#include "modules/models/behavior/behavior_model.hpp"
#include "modules/models/behavior/constant_velocity/constant_velocity.hpp"
#include "modules/models/dynamic/single_track.hpp"
#include "modules/models/execution/interpolation/interpolate.hpp"

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
      .def_property("local_map", &Agent::get_local_map, &Agent::set_local_map)
      .def_property_readonly("history", &Agent::get_state_input_history)
      .def_property_readonly("shape", &Agent::get_shape)
      .def_property_readonly("id", &Agent::get_agent_id)
      .def_property_readonly("followed_trajectory", &Agent::get_execution_trajectory)
      .def_property_readonly("planned_trajectory", &Agent::get_behavior_trajectory)
      .def_property_readonly("behavior_model", &Agent::get_behavior_model)
      .def_property_readonly("execution_model", &Agent::get_execution_model)
      .def_property_readonly("dynamic_model", &Agent::get_dynamic_model)
      .def_property_readonly("model3d", &Agent::get_model_3d)
      .def_property_readonly("state", &Agent::get_current_state)
      .def_property_readonly("goal_lane_id", &Agent::get_goal_lane_id)
      .def(py::pickle(
        [](const Agent& a) -> py::tuple { // __getstate__
            /* Return a tuple that fully encodes the state of the object */
            return py::make_tuple(a.get_local_map(), // 0
                                  a.get_state_input_history(), // 1
                                  a.get_shape(), // 2
                                  a.get_agent_id(), // 3
                                  a.get_execution_trajectory(), // 4
                                  a.get_behavior_trajectory(), // 5
                                  a.get_behavior_model(), // 6
                                  a.get_execution_model(), // 7
                                  a.get_dynamic_model(), // 8
                                  a.get_current_state(), // 9
                                  a.get_goal_lane_id()); // 10
        },
        [](py::tuple t) { // __setstate__
            if (t.size() != 11)
                throw std::runtime_error("Invalid agent state!");

            using modules::models::behavior::BehaviorConstantVelocity;
            using modules::models::dynamic::SingleTrackModel;
            using modules::models::execution::ExecutionModelInterpolate;
            using modules::world::map::LocalMap;


            /* Create a new C++ instance */
            Agent agent(t[9].cast<State>(),
                    std::make_shared<BehaviorConstantVelocity>(t[6].cast<BehaviorConstantVelocity>()), // todo resolve polymorphism
                    std::make_shared<SingleTrackModel>(t[8].cast<SingleTrackModel>()), // todo resolve polymorphism
                    std::make_shared<ExecutionModelInterpolate>(t[7].cast<ExecutionModelInterpolate>()), // todo resolve polymorphism
                    t[2].cast<modules::geometry::Polygon>(),
                    nullptr, // we have to set the params object afterwards as it relies on a python object
                    t[10].cast<LaneId>());
            agent.set_agent_id(t[3].cast<AgentId>());
            agent.set_local_map(std::make_shared<LocalMap>(t[0].cast<LocalMap>()));
            return agent;
            // todo: deserialize planned, followed trajectory and map interface
        }));

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
