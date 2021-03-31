// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "boost/variant.hpp"
#include "gtest/gtest.h"

#include "bark/commons/params/setter_params.hpp"
#include "bark/models/behavior/behavior_rss/behavior_rss.hpp"
#include "bark/models/behavior/behavior_safety/behavior_safety.hpp"
#include "bark/models/behavior/idm/idm_classic.hpp"
#include "bark/models/behavior/idm/idm_lane_tracking.hpp"
#include "bark/models/dynamic/single_track.hpp"
#include "bark/models/execution/interpolation/interpolate.hpp"
#include "bark/world/evaluation/base_evaluator.hpp"
#include "bark/world/evaluation/rss/evaluator_rss.hpp"
#include "bark/world/observed_world.hpp"
#include "bark/world/tests/make_test_world.hpp"
#include "bark/world/tests/make_test_xodr_map.hpp"

using bark::commons::SetterParams;
using bark::geometry::Model3D;
using bark::geometry::Polygon;
using bark::world::Agent;
using bark::world::ObservedWorld;
using bark::world::World;
using bark::world::WorldPtr;
using bark::world::evaluation::BaseEvaluator;
using bark::world::evaluation::EvaluationReturn;
using bark::world::evaluation::EvaluatorRSS;
using bark::world::goal_definition::GoalDefinitionPolygon;
using bark::world::goal_definition::GoalDefinitionPtr;
using bark::world::objects::AgentPtr;
using bark::world::tests::make_test_world;
using bark::world::tests::MakeXodrMapOneRoadTwoLanes;

using namespace bark::models::dynamic;
using namespace bark::models::execution;
using namespace bark::models::behavior;
using namespace bark::world::map;


TEST(behavior_rss, behavior_rss_system_test) {
  // Test-strategy: both time the ego agent is controlled by the
  // BehaviorIDMLaneTracking and attempts to change from the right to the
  // left lane (by setting the target corridor). One time the dummy rss
  // evaluator intervenes and the lane-change is aborted.
  auto params = std::make_shared<SetterParams>();

  // First case, we start with the desired velocity. After num steps, we should
  // advance
  float ego_velocity = 0.0, rel_distance = 7.0, velocity_difference = 0.0;
  float time_step = 0.2;

  //1 Create world with one agent in front 1
  WorldPtr world =
      make_test_world(0, rel_distance, ego_velocity, velocity_difference);

  //2 Set observer parameters for x variance to certain value 


  //3 Create behavior and plan


  //4 Get Current Expected violation
  // vehicle x and y aligned along x
  // x ------- y 
  // With some uncertainy
  // x --y , x----y , x ------------y, collision: xy
  // Probability that the deviation is in between two values giving a collision
  // CDF(first val < deviation < second val)

  // EXPECT_NEAR: current expected violation must be equal to calculated probaiblity
}

int main(int argc, char** argv) {
  // FLAGS_v = 5;
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}