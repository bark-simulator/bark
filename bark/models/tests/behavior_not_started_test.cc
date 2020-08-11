// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "boost/variant.hpp"
#include "gtest/gtest.h"

#include "bark/models/behavior/not_started/behavior_not_started.hpp"
#include "bark/world/observed_world.hpp"
#include "bark/world/tests/make_test_world.hpp"

using bark::models::behavior::BehaviorNotStarted;
using bark::models::behavior::BehaviorStatus;
using bark::world::Agent;
using bark::world::ObservedWorld;
using bark::world::World;
using bark::world::WorldPtr;

TEST(behavior_static_trajectory_plan, plan) {
  BehaviorNotStarted model(nullptr);
  auto observed_world =
      bark::world::tests::make_test_observed_world(0, 0, 0, 0);

  // Return all
  auto traj = model.Plan(3, observed_world);
  ASSERT_EQ(model.GetBehaviorStatus(), BehaviorStatus::NOT_STARTED_YET);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}