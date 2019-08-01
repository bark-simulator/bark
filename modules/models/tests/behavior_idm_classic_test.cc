// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include <Eigen/Core>
#include "gtest/gtest.h"

#include "modules/geometry/polygon.hpp"
#include "modules/geometry/line.hpp"
#include "modules/geometry/commons.hpp"
#include "modules/models/behavior/idm/idm_classic.hpp"
#include "modules/commons/params/default_params.hpp"
#include "modules/world/observed_world.hpp"

using namespace modules::models::dynamic;
using namespace modules::models::execution;
using namespace modules::commons;
using namespace modules::models::behavior;
using namespace modules::world::map;
using namespace modules::models::dynamic;
using namespace modules::world;
using namespace modules::geometry;

ObservedWorld make_test_observed_world(int num_other_agents, double rel_distance, double ego_velocity, double velocity_difference) {
  DefaultParams params;
  ExecutionModelPtr exec_model(nullptr);
  DynamicModelPtr dyn_model(nullptr);
  BehaviorModelPtr beh_model(new BehaviorIDMClassic(&params));

  Polygon polygon(Pose(1.25, 1, 0), std::vector<Point2d>{Point2d(0, 0), Point2d(0, 2), Point2d(4, 2), Point2d(4, 0), Point2d(0, 0)});
  
  State init_state1(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state1 << 0.0, 1.0, 0.0, 0.0, ego_velocity;
  AgentPtr agent1(new Agent(init_state1, beh_model, dyn_model, exec_model, polygon, &params));

  State init_state2(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  float rel_dist_vlength = rel_distance + polygon.front_dist_ + polygon.rear_dist_;
  init_state2 << 0.0, 1.0+rel_dist_vlength, 0.0, 0.0, ego_velocity - velocity_difference;
  AgentPtr agent2(new Agent(init_state2, beh_model, dyn_model, exec_model, polygon, &params));

  State init_state3(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state3 << 0.0, 10.0+rel_dist_vlength, 0.0, 0.0, ego_velocity - velocity_difference;
  AgentPtr agent3(new Agent(init_state3, beh_model, dyn_model, exec_model, polygon, &params));

  WorldPtr world(new World(&params));
  world->add_agent(agent1);
  if(num_other_agents == 1) {
    world->add_agent(agent2);
  } else if (num_other_agents == 2) {
    world->add_agent(agent3);
  }
  world->UpdateAgentRTree();

  // Define some driving corridor from x=1 to x=20 and add to local map of first agent
    Line center;
  center.add_point(Point2d(1,1));
  center.add_point(Point2d(2,1));
  center.add_point(Point2d(20,1));

  Line outer;
  outer.add_point(Point2d(1,2));
  outer.add_point(Point2d(2,2));
  outer.add_point(Point2d(20,2));

  Line inner;
  inner.add_point(Point2d(1,0));
  inner.add_point(Point2d(2,0));
  inner.add_point(Point2d(20,0));

  DrivingCorridor corridor(outer, inner, center);
  LocalMapPtr local_map(new LocalMap(0, GoalDefinition(), corridor));
  agent1->set_local_map(local_map);
  agent1->UpdateDrivingCorridor(20);

  // Create observed world for first agent
  WorldPtr current_world_state(world->Clone());
  ObservedWorld observed_world(*current_world_state, agent1->get_agent_id());

  return observed_world;
}


TEST(free_road_term, behavior_idm_classic) {
  DefaultParams params;
  BehaviorIDMClassic behavior(&params);
  const float desired_velocity = behavior.get_desired_velocity();
  const float max_acceleration = behavior.get_max_acceleration();
  const int exponent = behavior.get_exponent();

  float ego_velocity = desired_velocity, rel_distance = 7.0, velocity_difference=0.0;
  auto observed_world = make_test_observed_world(0,rel_distance, ego_velocity, velocity_difference);
  double idm_acceleration  = behavior.CalculateLongitudinalAcceleration(observed_world);

  // no vehicle is in front only free road term should give acceleration
  double desired_acceleration = max_acceleration*(1-pow(ego_velocity/desired_velocity, exponent));
  EXPECT_EQ(idm_acceleration, desired_acceleration);

  ego_velocity = desired_velocity+10;
  observed_world = make_test_observed_world(0,rel_distance, ego_velocity, velocity_difference);
  idm_acceleration  = behavior.CalculateLongitudinalAcceleration(observed_world);

  // no vehicle is in front only free road term should give acceleration
  desired_acceleration = max_acceleration*(1-pow(ego_velocity/desired_velocity, exponent));
  EXPECT_EQ(idm_acceleration, desired_acceleration);

  ego_velocity = desired_velocity-12;
  observed_world = make_test_observed_world(0,rel_distance, ego_velocity, velocity_difference);
  idm_acceleration  = behavior.CalculateLongitudinalAcceleration(observed_world);

  // no vehicle is in front only free road term should give acceleration
  desired_acceleration = max_acceleration*(1-pow(ego_velocity/desired_velocity, exponent));
  EXPECT_EQ(idm_acceleration, desired_acceleration);
}

TEST(interaction_term, behavior_idm_classic) {
  DefaultParams params;
  BehaviorIDMClassic behavior(&params);
  const float desired_velocity = behavior.get_desired_velocity();
  const float minimum_spacing = behavior.get_minimum_spacing();
  const float desired_time_headway = behavior.get_desired_time_headway();
  const float max_acceleration = behavior.get_max_acceleration();
  const float comfortable_braking_acceleration = behavior.get_comfortable_braking_acceleration();
  const int exponent = behavior.get_exponent();

  // vehicle is in front, zero velocity equal desired velocity thus only interaction term
  float ego_velocity = desired_velocity, rel_distance = 7.0, velocity_difference=0.0;
  auto observed_world = make_test_observed_world(1,rel_distance, ego_velocity, velocity_difference);
  double idm_acceleration  = behavior.CalculateLongitudinalAcceleration(observed_world);
  double helper_state = minimum_spacing + ego_velocity*desired_time_headway +
               ego_velocity*velocity_difference/(2*sqrt(max_acceleration*comfortable_braking_acceleration));
  double desired_acceleration = - max_acceleration*pow(helper_state/rel_distance, 2);
  EXPECT_EQ(idm_acceleration, desired_acceleration);

  
  // velocity difference to other vehicle
  ego_velocity = desired_velocity, rel_distance = 7.0, velocity_difference=5.0;
  observed_world = make_test_observed_world(1,rel_distance, ego_velocity, velocity_difference);
  idm_acceleration  = behavior.CalculateLongitudinalAcceleration(observed_world);
  helper_state = minimum_spacing + ego_velocity*desired_time_headway +
               ego_velocity*velocity_difference/(2*sqrt(max_acceleration*comfortable_braking_acceleration));
  desired_acceleration = - max_acceleration*pow(helper_state/rel_distance, 2);
  EXPECT_NEAR(idm_acceleration, desired_acceleration,0.001);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}