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
#include "modules/models/execution/interpolation/interpolate.hpp"
#include "modules/models/behavior/constant_velocity/constant_velocity.hpp"
#include "modules/models/tests/make_test_world.hpp"

using namespace modules::models::dynamic;
using namespace modules::models::execution;
using namespace modules::commons;
using namespace modules::models::behavior;
using namespace modules::world::map;
using namespace modules::models::dynamic;
using namespace modules::world;
using namespace modules::geometry;
using namespace modules::models::tests;



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
  EXPECT_NEAR(idm_acceleration, desired_acceleration, 0.0001);

  ego_velocity = desired_velocity+10;
  observed_world = make_test_observed_world(0,rel_distance, ego_velocity, velocity_difference);
  idm_acceleration  = behavior.CalculateLongitudinalAcceleration(observed_world);

  // no vehicle is in front only free road term should give acceleration
  desired_acceleration = max_acceleration*(1-pow(ego_velocity/desired_velocity, exponent));
  EXPECT_NEAR(idm_acceleration, desired_acceleration, 0.0001);

  ego_velocity = desired_velocity-12;
  observed_world = make_test_observed_world(0,rel_distance, ego_velocity, velocity_difference);
  idm_acceleration  = behavior.CalculateLongitudinalAcceleration(observed_world);

  // no vehicle is in front only free road term should give acceleration
  desired_acceleration = max_acceleration*(1-pow(ego_velocity/desired_velocity, exponent));
  EXPECT_NEAR(idm_acceleration, desired_acceleration, 0.0001);
}

TEST(interaction_term, behavior_idm_classic) {
  DefaultParams params;
  BehaviorIDMClassic behavior(&params);
  const float desired_velocity = behavior.get_desired_velocity();
  const float minimum_spacing = behavior.get_minimum_spacing();
  const float desired_time_headway = behavior.get_desired_time_headway();
  const float max_acceleration = behavior.get_max_acceleration();
  const float comfortable_braking_acceleration = behavior.get_comfortable_braking_acceleration();

  // vehicle is in front, zero velocity equal desired velocity thus only interaction term
  float ego_velocity = desired_velocity, rel_distance = 7.0, velocity_difference=0.0;
  auto observed_world = make_test_observed_world(1,rel_distance, ego_velocity, velocity_difference);
  double idm_acceleration  = behavior.CalculateLongitudinalAcceleration(observed_world);
  double helper_state = minimum_spacing + ego_velocity*desired_time_headway +
               ego_velocity*velocity_difference/(2*sqrt(max_acceleration*comfortable_braking_acceleration));
  double desired_acceleration = - max_acceleration*pow(helper_state/rel_distance, 2);
  EXPECT_NEAR(idm_acceleration, desired_acceleration, 0.001);

  
  // velocity difference to other vehicle
  ego_velocity = desired_velocity, rel_distance = 7.0, velocity_difference=5.0;
  observed_world = make_test_observed_world(1,rel_distance, ego_velocity, velocity_difference);
  idm_acceleration  = behavior.CalculateLongitudinalAcceleration(observed_world);
  helper_state = minimum_spacing + ego_velocity*desired_time_headway +
               ego_velocity*velocity_difference/(2*sqrt(max_acceleration*comfortable_braking_acceleration));
  desired_acceleration = - max_acceleration*pow(helper_state/rel_distance, 2);
  EXPECT_NEAR(idm_acceleration, desired_acceleration, 0.001);
}

TEST(drive_free, behavior_idm_classic) {
  DefaultParams params;
  BehaviorIDMClassic behavior(&params);
  const float desired_velocity = behavior.get_desired_velocity();

  // First case, we start with the desired velocity. After num steps, we should advance 
  float ego_velocity = desired_velocity, rel_distance = 7.0, velocity_difference=0.0;
  float time_step=0.2f;
  int num_steps = 10;
  WorldPtr world = make_test_world(0,rel_distance, ego_velocity, velocity_difference);

  float x_start = world->get_agents().begin()->second->get_current_state()[StateDefinition::X_POSITION];
  for (int i=0; i<num_steps; ++i ) {
    world->Step(time_step);
  }
  float x_end = world->get_agents().begin()->second->get_current_state()[StateDefinition::X_POSITION];
  float x_diff_desired = x_start + ego_velocity*num_steps*time_step - x_end;
  EXPECT_NEAR(x_diff_desired,0, 0.01);

}

TEST(drive_leading_vehicle, behavior_idm_classic) {
  DefaultParams params;
  BehaviorIDMClassic behavior(&params);
  const float desired_velocity = behavior.get_desired_velocity();
  const float minimum_spacing = behavior.get_minimum_spacing();
  const float desired_time_headway = behavior.get_desired_time_headway();

  // First case, we start with the desired velocity. After num steps, we should advance 
  float ego_velocity = desired_velocity, rel_distance = 5.0, velocity_difference=10;
  float time_step=0.2f; // Very small time steps to verify differential integration character
  int num_steps = 10;
  WorldPtr world = make_test_world(1,rel_distance, ego_velocity, velocity_difference);

 /* float v_start = world->get_agents().begin()->second->get_current_state()[StateDefinition::VEL_POSITION];
  for (int i=0; i<num_steps; ++i ) {
    world->Step(time_step);
  }
  float v_end = world->get_agents().begin()->second->get_current_state()[StateDefinition::VEL_POSITION];

  EXPECT_NEAR(v_end,ego_velocity-velocity_difference, 0.01);
*/
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}