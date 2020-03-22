// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include "modules/world/world.hpp"
#include "modules/world/observed_world.hpp"
#include "modules/world/map/roadgraph.hpp"
#include "modules/world/tests/make_test_world.hpp"
#include "modules/models/behavior/behavior_model.hpp"
#include "modules/models/behavior/idm/idm_classic.hpp"
#include "modules/models/behavior/dynamic_model/dynamic_model.hpp"
#include "python/world/world.hpp"
#include "python/world/agent.hpp"
#include "python/world/map.hpp"
#include "python/world/opendrive.hpp"
#include "python/world/goal_definition.hpp"
#include "python/world/evaluation.hpp"

namespace py = pybind11;
using namespace modules::world::objects;
using namespace modules::world::map;
using namespace modules::world::opendrive;
using modules::world::World;
using modules::models::behavior::DynamicBehaviorModel;
using modules::models::behavior::BehaviorIDMClassic;
using modules::world::WorldPtr;
using modules::world::ObservedWorldPtr;
using modules::commons::ParamsPtr;
using modules::models::behavior::Action;


void python_world(py::module m) {
  py::class_<World, std::shared_ptr<World>>(m, "World")
    .def(py::init<ParamsPtr>())
    .def("Step", &World::Step)
    .def("DoPlanning", &World::DoPlanning)
    .def("DoExecution", &World::DoExecution)
    .def("Observe", &World::Observe)
    .def("AddAgent", &World::AddAgent)
    .def("AddObject", &World::AddObject)
    .def("GetParams", &World::GetParams)
    .def("UpdateAgentRTree", &World::UpdateAgentRTree)
    .def("ClearEvaluators", &World::ClearEvaluators)
    .def("SetMap", &World::SetMap)
    .def("AddEvaluator", &World::AddEvaluator)
    .def("GetNearestAgents", &World::GetNearestAgents)
    .def_property_readonly("evaluators", &World::GetEvaluators)
    .def("Evaluate", &World::Evaluate)
    .def_property_readonly("agents", &World::GetAgents)
    .def_property_readonly("objects", &World::GetObjects)
    .def_property("time", &World::GetWorldTime, &World::SetWorldTime)
    .def_property_readonly("bounding_box", &World::BoundingBox)
    .def("GetAgent", &World::GetAgent)
    .def_property("map", &World::GetMap, &World::SetMap)
    .def("Copy", &World::Clone)
    .def("WorldExecutionAtTime", &World::WorldExecutionAtTime)
    .def("__repr__", [](const World& a) {
      return "bark.world.World";
    });

  m.def("MakeTestWorldHighway",
    &modules::world::tests::MakeTestWorldHighway);

  py::class_<ObservedWorld, World, std::shared_ptr<ObservedWorld>>(m, "ObservedWorld")
    .def(py::init<const WorldPtr&, const AgentId&>())
    .def_property_readonly("ego_agent", &ObservedWorld::GetEgoAgent)
    .def("Evaluate", &ObservedWorld::Evaluate)
    .def("GetAgentInFront", &ObservedWorld::GetAgentInFront)
    .def("GetAgentBehind", &ObservedWorld::GetAgentBehind)
    .def_property_readonly("road_corridor", &ObservedWorld::GetRoadCorridor)
    .def_property_readonly("ego_agent", &ObservedWorld::GetEgoAgent)
    .def_property_readonly("lane_corridor", &ObservedWorld::GetLaneCorridor)
    .def_property_readonly("other_agents", &ObservedWorld::GetOtherAgents)
    .def_property_readonly("ego_state", &ObservedWorld::CurrentEgoState)
    .def_property_readonly("ego_position", &ObservedWorld::CurrentEgoPosition)
    .def("PredictWithOthersIDM",
      &ObservedWorld::Predict<BehaviorIDMClassic, DynamicBehaviorModel>)
    .def_property_readonly("other_agents", &ObservedWorld::GetOtherAgents)
    .def("__repr__", [](const ObservedWorld& a) {
      return "bark.world.ObservedWorld";
    });

  py::class_<vertex_t>(m, "vertex_t")
    .def(py::init<>());

  py::class_<edge_t>(m, "edge_t")
    .def(py::init<>());

  py::class_<XodrLaneVertex, std::shared_ptr<XodrLaneVertex>>(
    m, "XodrLaneVertex")
    .def(py::init<int, int, XodrLanePtr>())
    .def_property_readonly("lane_id", &XodrLaneVertex::GetGlobalLineId)
    .def_property_readonly("lane", &XodrLaneVertex::GetLane);

  py::enum_<XodrLaneEdgeType>(m, "XodrLaneEdgeType")
    .value("LANE_SUCCESSOR_EDGE", XodrLaneEdgeType::LANE_SUCCESSOR_EDGE)
    .value("INNER_NEIGHBOR_EDGE", XodrLaneEdgeType::INNER_NEIGHBOR_EDGE)
    .value("OUTER_NEIGHBOR_EDGE", XodrLaneEdgeType::OUTER_NEIGHBOR_EDGE)
    .export_values();

  py::class_<XodrLaneEdge, std::shared_ptr<XodrLaneEdge>>(m, "XodrLaneEdge")
    .def(py::init<XodrLaneEdgeType>())
    .def_property_readonly("edge_type", &XodrLaneEdge::GetEdgeType);

  python_goal_definition(
    m.def_submodule("goal_definition", "agent goal definitions"));

  python_agent(m.def_submodule("agent", "Agent wrapping"));

  python_opendrive(m.def_submodule("opendrive", "OpenDrive wrapping"));

  python_map(m.def_submodule("map", "mapInterface wrapping"));

  python_evaluation(m.def_submodule("evaluation", "evaluators"));
}
