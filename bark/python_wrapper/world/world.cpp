// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/world/world.hpp"
#include "bark/models/behavior/behavior_model.hpp"
#include "bark/models/behavior/dynamic_model/dynamic_model.hpp"
#include "bark/models/behavior/idm/idm_classic.hpp"
#include "bark/python_wrapper/world/agent.hpp"
#include "bark/python_wrapper/world/evaluation.hpp"
#include "bark/python_wrapper/world/goal_definition.hpp"
#include "bark/python_wrapper/world/map.hpp"
#include "bark/python_wrapper/world/opendrive.hpp"
#include "bark/python_wrapper/world/world.hpp"
#include "bark/world/map/roadgraph.hpp"
#include "bark/world/observed_world.hpp"
#include "bark/world/tests/make_test_world.hpp"

namespace py = pybind11;
using namespace bark::world::objects;
using namespace bark::world::map;
using namespace bark::world::opendrive;
using bark::commons::ParamsPtr;
using bark::models::behavior::Action;
using bark::models::behavior::BehaviorDynamicModel;
using bark::models::behavior::BehaviorIDMClassic;
using bark::world::ObservedWorldPtr;
using bark::world::World;
using bark::world::WorldPtr;

void python_world(py::module m) {
  py::class_<World, std::shared_ptr<World>>(m, "World")
      .def(py::init<ParamsPtr>())
      .def("Step", &World::Step)
      .def("PlanAgents", &World::PlanAgents)
      .def("Execute", &World::Execute)
      .def("Observe", &World::Observe)
      .def("AddAgent", &World::AddAgent)
      .def("RemoveAgentById", &World::RemoveAgentById)
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
      .def_property_readonly("agents_valid", &World::GetValidAgents)
      .def_property_readonly("objects", &World::GetObjects)
      .def_property("time", &World::GetWorldTime, &World::SetWorldTime)
      .def_property_readonly("bounding_box", &World::BoundingBox)
      .def("GetAgent", &World::GetAgent)
      .def_property("map", &World::GetMap, &World::SetMap)
      .def("Copy", &World::Clone)
      .def("GetWorldAtTime", &World::GetWorldAtTime)
      // .def("FillWorldFromCarla",&World::FillWorldFromCarla)
      // .def("PlanAgents",&World::PlanSpecificAgents)
      .def("__repr__", [](const World& a) { return "bark.core.world.World"; });

  m.def("MakeTestWorldHighway", &bark::world::tests::MakeTestWorldHighway);

  py::class_<ObservedWorld, World, std::shared_ptr<ObservedWorld>>(
      m, "ObservedWorld")
      .def(py::init<const WorldPtr&, const AgentId&>())
      .def_property_readonly("ego_agent", &ObservedWorld::GetEgoAgent)
      .def("Evaluate", &ObservedWorld::Evaluate)
      .def("GetAgentInFront",
           py::overload_cast<>(&ObservedWorld::GetAgentInFront, py::const_))
      .def("GetAgentInFront", py::overload_cast<const LaneCorridorPtr&>(
                                  &ObservedWorld::GetAgentInFront, py::const_))
      .def("GetAgentBehind",
           py::overload_cast<>(&ObservedWorld::GetAgentBehind, py::const_))
      .def("GetAgentBehind", py::overload_cast<const LaneCorridorPtr&>(
                                 &ObservedWorld::GetAgentBehind, py::const_))
      .def_property_readonly("road_corridor", &ObservedWorld::GetRoadCorridor)
      .def_property_readonly("ego_agent", &ObservedWorld::GetEgoAgent)
      .def_property_readonly("lane_corridor", &ObservedWorld::GetLaneCorridor)
      .def_property_readonly("other_agents", &ObservedWorld::GetOtherAgents)
      .def_property_readonly("ego_state", &ObservedWorld::CurrentEgoState)
      .def_property_readonly("ego_position", &ObservedWorld::CurrentEgoPosition)
      .def("PredictWithOthersIDM",
           &ObservedWorld::Predict<BehaviorIDMClassic, BehaviorDynamicModel>)
      .def_property_readonly("other_agents", &ObservedWorld::GetOtherAgents)
      .def("__repr__", [](const ObservedWorld& a) {
        return "bark.core.world.ObservedWorld";
      });

  py::class_<vertex_t>(m, "vertex_t").def(py::init<>());

  py::class_<edge_t>(m, "edge_t").def(py::init<>());

  py::class_<XodrLaneVertex, std::shared_ptr<XodrLaneVertex>>(m,
                                                              "XodrLaneVertex")
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
