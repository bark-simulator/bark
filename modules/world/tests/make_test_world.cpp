// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/world/tests/make_test_world.hpp"
#include <vector>
#include "modules/commons/params/default_params.hpp"
#include "modules/geometry/commons.hpp"
#include "modules/geometry/line.hpp"
#include "modules/geometry/polygon.hpp"
#include "modules/models/behavior/constant_velocity/constant_velocity.hpp"
#include "modules/models/behavior/idm/idm_classic.hpp"
#include "modules/models/dynamic/single_track.hpp"
#include "modules/models/execution/interpolation/interpolate.hpp"
#include "modules/world/observed_world.hpp"
#include "modules/world/tests/make_test_xodr_map.hpp"

using namespace modules::models::dynamic;
using namespace modules::models::execution;
using namespace modules::commons;
using namespace modules::models::behavior;
using namespace modules::world::map;
using namespace modules::models::dynamic;
using namespace modules::world::goal_definition;
using namespace modules::world::tests;

using modules::geometry::Model3D;
using modules::geometry::Point2d;
using modules::geometry::Polygon;
using modules::geometry::Pose;
using modules::world::World;
using modules::world::WorldPtr;
using modules::world::goal_definition::GoalDefinitionPolygon;
using modules::world::goal_definition::GoalDefinitionPtr;
using modules::world::map::MapInterface;
using modules::world::map::MapInterfacePtr;
using modules::world::objects::Agent;
using modules::world::objects::AgentPtr;
using modules::world::opendrive::OpenDriveMapPtr;
using modules::world::tests::MakeXodrMapOneRoadTwoLanes;

WorldPtr modules::world::tests::make_test_world(
    int num_other_agents, double rel_distance, double ego_velocity,
    double velocity_difference, const GoalDefinitionPtr& ego_goal_definition) {
  float pos_x = 3.0;
  float pos_y = -1.75;

  OpenDriveMapPtr open_drive_map = MakeXodrMapOneRoadTwoLanes();

  MapInterfacePtr map_interface = std::make_shared<MapInterface>();
  map_interface->interface_from_opendrive(open_drive_map);

  auto params = std::make_shared<DefaultParams>();

  ExecutionModelPtr exec_model(new ExecutionModelInterpolate(params));
  DynamicModelPtr dyn_model(nullptr);
  BehaviorModelPtr beh_model_idm(new BehaviorIDMClassic(params));
  BehaviorModelPtr beh_model_const(new BehaviorConstantVelocity(params));

  Polygon polygon(
      Pose(1.25, 1, 0),
      std::vector<Point2d>{Point2d(-1, -1), Point2d(-1, 1), Point2d(3, 1),
                           Point2d(3, -1), Point2d(-1, -1)});

  State init_state1(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state1 << 0.0, pos_x, pos_y, 0.0, ego_velocity;
  AgentPtr agent1(new Agent(init_state1, beh_model_idm, dyn_model, exec_model,
                            polygon, params, ego_goal_definition,
                            map_interface, geometry::Model3D()));

  State init_state2(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  float rel_dist_vlength =
      rel_distance + polygon.front_dist_ + polygon.rear_dist_;  // NOLINT
  init_state2 << 0.0, pos_x + rel_dist_vlength, pos_y, 0.0,
      ego_velocity - velocity_difference;  // NOLINT
  AgentPtr agent2(new Agent(init_state2, beh_model_const, dyn_model, exec_model,
                            polygon, params, ego_goal_definition,
                            map_interface, geometry::Model3D()));

  State init_state3(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state3 << 0.0, pos_x + 10.0 + rel_dist_vlength, pos_y, 0.0,
      ego_velocity - velocity_difference;  // NOLINT
  AgentPtr agent3(new Agent(init_state3, beh_model_const, dyn_model, exec_model,
                            polygon, params, ego_goal_definition,
                            map_interface,
                            geometry::Model3D()));  // NOLINT

  WorldPtr world(new World(params));
  world->AddAgent(agent1);
  if (num_other_agents == 1) {
    world->AddAgent(agent2);
  } else if (num_other_agents == 2) {
    world->AddAgent(agent3);
  }
  world->UpdateAgentRTree();

  world->SetMap(map_interface);
  return WorldPtr(world->Clone());
}

ObservedWorld modules::world::tests::make_test_observed_world(
    int num_other_agents, double rel_distance, double ego_velocity,
    double velocity_difference, const GoalDefinitionPtr& ego_goal_definition) {
  // Create observed world for first agent
  WorldPtr current_world_state = make_test_world(
      num_other_agents, rel_distance, ego_velocity, velocity_difference,
      ego_goal_definition);
  ObservedWorld observed_world(
      current_world_state,
      current_world_state->GetAgents().begin()->second->GetAgentId());
  return observed_world;
}

WorldPtr modules::world::tests::MakeTestWorldHighway() {
  using modules::commons::DefaultParams;

  using modules::geometry::standard_shapes::CarRectangle;
  using StateDefinition::MIN_STATE_SIZE;

  auto params = std::make_shared<DefaultParams>();

  // Setting Up Map
  OpenDriveMapPtr open_drive_map = MakeXodrMapOneRoadTwoLanes();
  MapInterfacePtr map_interface = std::make_shared<MapInterface>();
  map_interface->interface_from_opendrive(open_drive_map);

  // Goal Definition
  Polygon polygon(
      Pose(1, 1, 0),
      std::vector<Point2d>{Point2d(0, 0), Point2d(0, 2), Point2d(2, 2),
                           Point2d(2, 0), Point2d(0, 0)});
  std::shared_ptr<Polygon> goal_polygon(
      std::dynamic_pointer_cast<Polygon>(polygon.Translate(Point2d(50, -2))));
  auto goal_ptr = std::make_shared<GoalDefinitionPolygon>(*goal_polygon);

  // Setting Up Agents (one in front of another)
  ExecutionModelPtr exec_model(new ExecutionModelInterpolate(params));
  DynamicModelPtr dyn_model(new SingleTrackModel(params));
  BehaviorModelPtr beh_model(new BehaviorConstantVelocity(params));
  Polygon car_polygon = CarRectangle();

  State init_state1(static_cast<int>(MIN_STATE_SIZE));
  init_state1 << 0.0, 3.0, -1.75, 0.0, 5.0;
  AgentPtr agent1(new Agent(init_state1, beh_model, dyn_model, exec_model,
                            car_polygon, params, goal_ptr, map_interface,
                            Model3D()));  // NOLINT

  State init_state2(static_cast<int>(MIN_STATE_SIZE));
  init_state2 << 0.0, 10.0, -1.75, 0.0, 5.0;
  AgentPtr agent2(new Agent(init_state2, beh_model, dyn_model, exec_model,
                            car_polygon, params, goal_ptr, map_interface,
                            Model3D()));  // NOLINT

  // Construct World
  WorldPtr world(new World(params));
  world->AddAgent(agent1);
  world->AddAgent(agent2);
  world->UpdateAgentRTree();

  State init_state3(static_cast<int>(MIN_STATE_SIZE));
  init_state3 << 0.0, 20.0, -1.75, 0.0, 5.0;
  AgentPtr agent3(new Agent(init_state3, beh_model, dyn_model, exec_model,
                            polygon, params, goal_ptr, map_interface,
                            Model3D()));  // NOLINT
  world->AddAgent(agent3);
  world->UpdateAgentRTree();

  // Adding a fourth agent in right lane
  State init_state4(static_cast<int>(MIN_STATE_SIZE));
  init_state4 << 0.0, 5.0, -5.25, 0.0, 5.0;
  AgentPtr agent4(new Agent(init_state4, beh_model, dyn_model, exec_model,
                            polygon, params, goal_ptr, map_interface,
                            Model3D()));  // NOLINT

  world->AddAgent(agent4);
  world->UpdateAgentRTree();

  return world;
}
