// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include "gtest/gtest.h"
#include "modules/models/dynamic/single_track.hpp"
#include "modules/models/execution/interpolation/interpolate.hpp"
#include "modules/models/execution/mpc/mpc.hpp"
#include "modules/world/map/map_interface.hpp"
#include "modules/world/map/roadgraph.hpp"
#include "modules/world/opendrive/opendrive.hpp"
#include "modules/world/map/local_map.hpp"
#include "modules/models/behavior/constant_velocity/constant_velocity.hpp"
#include "modules/geometry/polygon.hpp"
#include "modules/commons/params/default_params.hpp"
#include "modules/world/objects/agent.hpp"
#include "modules/world/observed_world.hpp"
#include "modules/world/evaluation/evaluator_collision_agents.hpp"
#include "modules/world/evaluation/evaluator_collision_driving_corridor.hpp"

using namespace modules::models::dynamic;
using namespace modules::geometry;
using namespace modules::commons;
using namespace modules::models::behavior;
using namespace modules::models::execution;
using namespace modules::world::opendrive;
using namespace modules::world::map;
using namespace modules::world::objects;
using namespace modules::world;
using namespace modules::world::evaluation;


TEST(observed_world, agent_in_front)
{
  DefaultParams params;
  ExecutionModelPtr exec_model(new ExecutionModelInterpolate(&params));
  DynamicModelPtr dyn_model(new SingleTrackModel());
  BehaviorModelPtr beh_model(new BehaviorConstantVelocity(&params));
  EvaluatorPtr col_checker(new EvaluatorCollisionAgents());

  Polygon polygon(Pose(1.25, 1, 0), std::vector<Point2d>{Point2d(0, 0), Point2d(0, 2), Point2d(4, 2), Point2d(4, 0), Point2d(0, 0)});
  
  State init_state1(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state1 << 0.0, 0.0, 0.0, 0.0, 5.0;
  AgentPtr agent1(new Agent(init_state1, beh_model, dyn_model, exec_model, polygon, &params));

  State init_state2(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state2 << 0.0, 1.0, 0.0, 0.0, 5.0;
  AgentPtr agent2(new Agent(init_state2, beh_model, dyn_model, exec_model, polygon, &params));

  WorldPtr world(new World(&params));
  world->add_agent(agent1);
  world->add_agent(agent2);
  world->UpdateAgentRTree();


  // Define some driving corridor and add to local map of agent
    Line center;
  center.add_point(Point2d(1,1));
  center.add_point(Point2d(2,1));
  center.add_point(Point2d(10,1));

  Line outer;
  outer.add_point(Point2d(1,2));
  outer.add_point(Point2d(2,2));
  outer.add_point(Point2d(10,2));

  Line inner;
  inner.add_point(Point2d(1,0));
  inner.add_point(Point2d(2,0));
  inner.add_point(Point2d(10,0));

  DrivingCorridor corridor(outer, inner, center);
  LocalMapPtr local_map(new LocalMap(0, GoalDefinition(), corridor));
  agent1->set_local_map(local_map);

  // Create observed world for this agent
  WorldPtr current_world_state(world->Clone());
  ObservedWorld observed_world(*current_world_state, agent1->get_agent_id());

  std::pair<AgentPtr, Frenet> leading_vehicle = observed_world.get_agent_in_front();
  EXPECT_TRUE(leading_vehicle.first != nullptr);


}




