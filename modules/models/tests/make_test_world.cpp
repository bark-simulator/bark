// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <vector>
#include "modules/models/tests/make_test_world.hpp"
#include "modules/geometry/polygon.hpp"
#include "modules/geometry/line.hpp"
#include "modules/geometry/commons.hpp"
#include "modules/models/behavior/idm/idm_classic.hpp"
#include "modules/commons/params/default_params.hpp"
#include "modules/world/observed_world.hpp"
#include "modules/models/execution/interpolation/interpolate.hpp"
#include "modules/models/behavior/constant_velocity/constant_velocity.hpp"
#include "modules/world/tests/make_test_xodr_map.hpp"

using namespace modules::models::dynamic;
using namespace modules::models::execution;
using namespace modules::commons;
using namespace modules::models::behavior;
using namespace modules::world::map;
using namespace modules::models::dynamic;
using namespace modules::world;
using namespace modules::geometry;
using namespace modules::world::goal_definition;
using namespace modules::world::tests;
using modules::world::goal_definition::GoalDefinitionPtr;
using modules::world::map::MapInterface;
using modules::world::map::MapInterfacePtr;

WorldPtr modules::models::tests::make_test_world(
  int num_other_agents, double rel_distance,
  double ego_velocity, double velocity_difference,
  const GoalDefinitionPtr& ego_goal_definition) {

  float pos_x = 3.0;
  float pos_y = -1.75;

  OpenDriveMapPtr open_drive_map = make_xodr_map_one_road_two_lanes();

  MapInterfacePtr map_interface = std::make_shared<MapInterface>();
  map_interface->interface_from_opendrive(open_drive_map);

  DefaultParams params;
  
  ExecutionModelPtr exec_model(new ExecutionModelInterpolate(&params));
  DynamicModelPtr dyn_model(nullptr);
  BehaviorModelPtr beh_model_idm(new BehaviorIDMClassic(&params));
  BehaviorModelPtr beh_model_const(new BehaviorConstantVelocity(&params));

  Polygon polygon(Pose(1.25, 1, 0),
    std::vector<Point2d>{Point2d(-1, -1),
                         Point2d(-1, 1),
                         Point2d(3, 1),
                         Point2d(3, -1),
                         Point2d(-1, -1)});

  State init_state1(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state1 << 0.0, pos_x, pos_y, 0.0, ego_velocity;
  AgentPtr agent1(new Agent(init_state1,
                            beh_model_idm,
                            dyn_model,
                            exec_model,
                            polygon,
                            &params,
                            ego_goal_definition,
                            map_interface,
                            geometry::Model3D()));

  State init_state2(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  float rel_dist_vlength = rel_distance + polygon.front_dist_ + polygon.rear_dist_;  // NOLINT
  init_state2 << 0.0, pos_x+rel_dist_vlength, pos_y, 0.0, ego_velocity - velocity_difference;  // NOLINT
  AgentPtr agent2(new Agent(init_state2,
                            beh_model_const,
                            dyn_model,
                            exec_model,
                            polygon,
                            &params,
                            ego_goal_definition,
                            map_interface,
                            geometry::Model3D()));

  State init_state3(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state3 << 0.0, pos_x+10.0+rel_dist_vlength, pos_y, 0.0, ego_velocity - velocity_difference;   // NOLINT
  AgentPtr agent3(new Agent(init_state3, beh_model_const, dyn_model, exec_model, polygon, &params,
                            ego_goal_definition,
                            map_interface,
                            geometry::Model3D()));  // NOLINT

  WorldPtr world(new World(&params));
  world->add_agent(agent1);
  if (num_other_agents == 1) {
    world->add_agent(agent2);
  } else if (num_other_agents == 2) {
    world->add_agent(agent3);
  }
  world->UpdateAgentRTree();

  world->set_map(map_interface);
  return WorldPtr(world->Clone());
}

ObservedWorld modules::models::tests::make_test_observed_world(
  int num_other_agents,
  double rel_distance,
  double ego_velocity,
  double velocity_difference,
  const GoalDefinitionPtr& ego_goal_definition) {
  // Create observed world for first agent
  WorldPtr current_world_state =
    modules::models::tests::make_test_world(num_other_agents,
    rel_distance,
    ego_velocity,
    velocity_difference,
    ego_goal_definition);
  ObservedWorld observed_world(
    current_world_state,
    current_world_state->get_agents().begin()->second->get_agent_id());
  return observed_world;
}
