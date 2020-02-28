// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <typeinfo>
#include <stdexcept>

#include "agent.hpp"
#include "python/polymorphic_conversion.hpp"
#include "modules/world/objects/agent.hpp"
#include "modules/world/objects/object.hpp"
#include "modules/models/dynamic/single_track.hpp"
#include "modules/models/execution/interpolation/interpolate.hpp"
#include "modules/world/goal_definition/goal_definition.hpp"
#include "modules/world/goal_definition/goal_definition_polygon.hpp"
#include "modules/world/goal_definition/goal_definition_state_limits.hpp"




namespace py = pybind11;
using namespace modules::world::objects;
using namespace modules::world::goal_definition;
using namespace modules::models::dynamic;
using namespace modules::commons;
using namespace modules::models::behavior;
using namespace modules::models::execution;
using namespace modules::geometry;

void python_agent(py::module m) {
  py::class_<Agent, AgentPtr>(m, "Agent")
    .def(
      py::init<const State &,
      const BehaviorModelPtr &,
      const DynamicModelPtr &,
      const ExecutionModelPtr &,
      const Polygon &, const ParamsPtr&,
      const GoalDefinitionPtr &,
      const MapInterfacePtr &,
      const Model3D &>(),
      py::arg("initial_state"),
      py::arg("behavior_model_ptr"),
      py::arg("dynamic_model_ptr"),
      py::arg("execution_model"),
      py::arg("shape"),
      py::arg("params"),
      py::arg("goal_definition") = nullptr,
      py::arg("map_interface") = nullptr,
      py::arg("model_3d") = Model3D())
    .def("__repr__", [](const Agent &a) {
      return "bark.agent.Agent";
    })
    .def_property_readonly("history", &Agent::GetStateInputHistory)
    .def_property_readonly("shape", &Agent::GetShape)
    .def_property_readonly("id", &Agent::GetAgentId)
    .def_property_readonly("followed_trajectory",
      &Agent::GetExecutionTrajectory)
    .def_property_readonly("planned_trajectory", &Agent::GetBehaviorTrajectory)
    .def_property("behavior_model",
      &Agent::GetBehaviorModel, &Agent::SetBehaviorModel)
    .def_property_readonly("execution_model", &Agent::GetExecutionModel)
    .def_property_readonly("dynamic_model", &Agent::GetDynamicModel)
    .def_property_readonly("model3d", &Agent::GetModel3d)
    .def_property_readonly("state", &Agent::GetCurrentState)
    .def_property_readonly("road_corridor", &Agent::GetRoadCorridor)
    .def_property("goal_definition",
      &Agent::GetGoalDefinition, &Agent::SetGoalDefinition)
    .def("SetAgentId", &Object::SetAgentId)
    .def("GenerateRoadCorridor", &Agent::GenerateRoadCorridor)
    .def(py::pickle(
      [](const Agent& a) -> py::tuple {
          return py::make_tuple(
            a.GetStateInputHistory(),  // 1
            a.GetShape(),  // 2
            a.GetAgentId(),  // 3
            a.GetExecutionTrajectory(),  // 4
            a.GetBehaviorTrajectory(),  // 5
            BehaviorModelToPython(a.GetBehaviorModel()),  // 6
            a.GetExecutionModel(),  // 7
            a.GetDynamicModel(),  // 8
            a.GetCurrentState(),  // 9
            GoalDefinitionToPython(a.GetGoalDefinition()));  // 10
      },
      [](py::tuple t) {
        if (t.size() != 10)
          throw std::runtime_error("Invalid agent state!");

        using modules::models::dynamic::SingleTrackModel;
        using modules::models::execution::ExecutionModelInterpolate;
        Agent agent(
          t[8].cast<State>(),
          PythonToBehaviorModel(t[5].cast<py::tuple>()),
          std::make_shared<SingleTrackModel>(t[7].cast<SingleTrackModel>()),
          std::make_shared<ExecutionModelInterpolate>(
            t[6].cast<ExecutionModelInterpolate>()),
          t[1].cast<modules::geometry::Polygon>(),
          nullptr,
          PythonToGoalDefinition(t[9].cast<py::tuple>()));
        agent.SetAgentId(t[2].cast<AgentId>());
        agent.SetStateInputHistory(t[0].cast<StateActionHistory>());
        return agent;
      }));

  py::class_<Object, ObjectPtr>(m, "Object")
    .def(
      py::init<const Polygon &,
      const ParamsPtr&,
      const Model3D &>(),
      py::arg("shape"),
      py::arg("params"),
      py::arg("model_3d") = Model3D())
    .def("__repr__", [](const Object &a) {
      return "bark.agent.Object";
    })
    .def_property_readonly("shape", &Object::GetShape)
    .def_property_readonly("id", &Object::GetAgentId)
    .def("SetAgentId", &Object::SetAgentId);
}
