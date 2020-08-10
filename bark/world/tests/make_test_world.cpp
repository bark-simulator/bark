// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/world/tests/make_test_world.hpp"
#include <vector>
#include "bark/commons/params/setter_params.hpp"
#include "bark/geometry/commons.hpp"
#include "bark/geometry/line.hpp"
#include "bark/geometry/polygon.hpp"
#include "bark/models/behavior/constant_acceleration/constant_acceleration.hpp"
#include "bark/models/behavior/idm/idm_classic.hpp"
#include "bark/models/dynamic/single_track.hpp"
#include "bark/models/execution/interpolation/interpolate.hpp"
#include "bark/world/observed_world.hpp"
#include "bark/world/tests/make_test_xodr_map.hpp"

using namespace bark::models::dynamic;
using namespace bark::models::execution;
using namespace bark::commons;
using namespace bark::models::behavior;
using namespace bark::world::map;
using namespace bark::models::dynamic;
using namespace bark::world::goal_definition;
using namespace bark::world::tests;

using bark::geometry::Model3D;
using bark::geometry::Point2d;
using bark::geometry::Polygon;
using bark::geometry::Pose;
using bark::world::World;
using bark::world::WorldPtr;
using bark::world::goal_definition::GoalDefinitionPolygon;
using bark::world::goal_definition::GoalDefinitionPtr;
using bark::geometry::standard_shapes::GenerateGoalRectangle;
using bark::world::map::MapInterface;
using bark::world::map::MapInterfacePtr;
using bark::world::objects::Agent;
using bark::world::objects::AgentPtr;
using bark::world::opendrive::OpenDriveMapPtr;
using bark::world::tests::MakeXodrMapOneRoadTwoLanes;

WorldPtr bark::world::tests::make_test_world(
    int num_other_agents, double rel_distance, double ego_velocity,
    double velocity_difference, const GoalDefinitionPtr& ego_goal_definition,
    float ego_acc, float other_acc) {
  float pos_x = 3.0;
  float pos_y = -1.75;

  OpenDriveMapPtr open_drive_map = MakeXodrMapOneRoadTwoLanes();

  MapInterfacePtr map_interface = std::make_shared<MapInterface>();
  map_interface->interface_from_opendrive(open_drive_map);

  auto params = std::make_shared<SetterParams>();

  ExecutionModelPtr exec_model(new ExecutionModelInterpolate(params));
  DynamicModelPtr dyn_model(new SingleTrackModel(params));
  BehaviorModelPtr beh_model_idm(new BehaviorIDMClassic(params));
  beh_model_idm->SetLastAction(Continuous1DAction(ego_acc));
  BehaviorModelPtr beh_model_const(new BehaviorConstantAcceleration(params));
  beh_model_const->SetLastAction(Continuous1DAction(other_acc));

  Polygon polygon(
      Pose(1.25, 1, 0),
      std::vector<Point2d>{Point2d(-1, -1), Point2d(-1, 1), Point2d(3, 1),
                           Point2d(3, -1), Point2d(-1, -1)});

  State init_state1(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state1 << 0.0, pos_x, pos_y, 0.0, ego_velocity;
  AgentPtr agent1(new Agent(init_state1, beh_model_idm, dyn_model, exec_model,
                            polygon, params, ego_goal_definition, map_interface,
                            geometry::Model3D()));
  agent1->SetAgentId(1);

  State init_state2(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  float rel_dist_vlength =
      rel_distance + polygon.front_dist_ + polygon.rear_dist_;  // NOLINT
  init_state2 << 0.0, pos_x + rel_dist_vlength, pos_y, 0.0,
      ego_velocity - velocity_difference;  // NOLINT
  AgentPtr agent2(new Agent(init_state2, beh_model_const, dyn_model, exec_model,
                            polygon, params, ego_goal_definition, map_interface,
                            geometry::Model3D()));
  agent2->SetAgentId(2);

  State init_state3(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state3 << 0.0, pos_x + 10.0 + rel_dist_vlength, pos_y, 0.0,
      ego_velocity - velocity_difference;  // NOLINT
  AgentPtr agent3(new Agent(init_state3, beh_model_const, dyn_model, exec_model,
                            polygon, params, ego_goal_definition, map_interface,
                            geometry::Model3D()));  // NOLINT
  agent3->SetAgentId(3);

  WorldPtr world(new World(params));
  world->AddAgent(agent1);
  if (num_other_agents >= 1) {
    world->AddAgent(agent2);
  }
  if (num_other_agents >= 2) {
    world->AddAgent(agent3);
  }
  world->UpdateAgentRTree();

  world->SetMap(map_interface);
  return WorldPtr(world->Clone());
}

ObservedWorld bark::world::tests::make_test_observed_world(
    int num_other_agents, double rel_distance, double ego_velocity,
    double velocity_difference, const GoalDefinitionPtr& ego_goal_definition,
    float ego_acc, float other_acc) {
  // Create observed world for first agent
  WorldPtr current_world_state = make_test_world(
      num_other_agents, rel_distance, ego_velocity, velocity_difference,
      ego_goal_definition, ego_acc, other_acc);
  ObservedWorld observed_world(
      current_world_state,
      current_world_state->GetAgents().begin()->second->GetAgentId());
  return observed_world;
}

WorldPtr bark::world::tests::MakeTestWorldHighway() {
  using bark::commons::SetterParams;

  using bark::geometry::standard_shapes::CarRectangle;
  using StateDefinition::MIN_STATE_SIZE;

  auto params = std::make_shared<SetterParams>();

  // Setting Up Map
  OpenDriveMapPtr open_drive_map = MakeXodrMapOneRoadTwoLanes();
  MapInterfacePtr map_interface = std::make_shared<MapInterface>();
  map_interface->interface_from_opendrive(open_drive_map);

  // Goal Definition
  Polygon polygon = GenerateGoalRectangle(6,3);
  std::shared_ptr<Polygon> goal_polygon(
      std::dynamic_pointer_cast<Polygon>(polygon.Translate(Point2d(50, -2))));
  auto goal_ptr = std::make_shared<GoalDefinitionPolygon>(*goal_polygon);

  // Setting Up Agents (one in front of another)
  ExecutionModelPtr exec_model(new ExecutionModelInterpolate(params));
  ExecutionModelPtr exec_model1(new ExecutionModelInterpolate(params));
  ExecutionModelPtr exec_model2(new ExecutionModelInterpolate(params));
  ExecutionModelPtr exec_model3(new ExecutionModelInterpolate(params));
  DynamicModelPtr dyn_model(new SingleTrackModel(params));
  BehaviorModelPtr beh_model(new BehaviorConstantAcceleration(params));
  BehaviorModelPtr beh_model1(new BehaviorConstantAcceleration(params));
  BehaviorModelPtr beh_model2(new BehaviorConstantAcceleration(params));
  BehaviorModelPtr beh_model3(new BehaviorConstantAcceleration(params));
  Polygon car_polygon = CarRectangle();

  State init_state1(static_cast<int>(MIN_STATE_SIZE));
  init_state1 << 0.0, 3.0, -1.75, 0.0, 5.0;
  AgentPtr agent1(new Agent(init_state1, beh_model, dyn_model, exec_model,
                            car_polygon, params, goal_ptr, map_interface,
                            Model3D()));  // NOLINT
  agent1->SetAgentId(1);

  State init_state2(static_cast<int>(MIN_STATE_SIZE));
  init_state2 << 0.0, 10.0, -1.75, 0.0, 5.0;
  AgentPtr agent2(new Agent(init_state2, beh_model1, dyn_model, exec_model1,
                            car_polygon, params, goal_ptr, map_interface,
                            Model3D()));  // NOLINT
  agent2->SetAgentId(2);

  // Construct World
  WorldPtr world(new World(params));
  world->AddAgent(agent1);
  world->AddAgent(agent2);
  world->UpdateAgentRTree();

  State init_state3(static_cast<int>(MIN_STATE_SIZE));
  init_state3 << 0.0, 20.0, -1.75, 0.0, 5.0;
  AgentPtr agent3(new Agent(init_state3, beh_model2, dyn_model, exec_model2,
                            polygon, params, goal_ptr, map_interface,
                            Model3D()));  // NOLINT
  agent3->SetAgentId(3);
  world->AddAgent(agent3);
  world->UpdateAgentRTree();

  // Adding a fourth agent in right lane
  State init_state4(static_cast<int>(MIN_STATE_SIZE));
  init_state4 << 0.0, 5.0, -5.25, 0.0, 5.0;
  AgentPtr agent4(new Agent(init_state4, beh_model3, dyn_model, exec_model3,
                            polygon, params, goal_ptr, map_interface,
                            Model3D()));  // NOLINT
  agent4->SetAgentId(4);
  world->AddAgent(agent4);
  world->UpdateAgentRTree();

  return world;
}
