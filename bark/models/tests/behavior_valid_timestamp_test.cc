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
#include "bark/models/behavior/idm/idm_classic.hpp"
#include "bark/models/behavior/rule_based/mobil_behavior.hpp"
#include "bark/world/observed_world.hpp"
#include "bark/world/tests/make_test_world.hpp"

using bark::commons::SetterParams;
using bark::models::behavior::BehaviorMobilRuleBased;
using bark::models::behavior::BehaviorStatus;
using bark::world::Agent;
using bark::world::ObservedWorld;
using bark::world::World;
using bark::world::WorldPtr;

TEST(behavior_mobil, first_valid_timestamp_zero) {
  auto params = std::make_shared<SetterParams>();
  BehaviorMobilRuleBased model(params);
  ASSERT_EQ(model.GetBehaviorStatus(), BehaviorStatus::NOT_STARTED_YET);

  auto observed_world =
      bark::world::tests::make_test_observed_world(0, 0, 0, 0);

  // Return all
  auto traj = model.Plan(1, observed_world);
  ASSERT_EQ(model.GetBehaviorStatus(), BehaviorStatus::VALID);
}

TEST(behavior_mobil, first_valid_timestamp_nonzero) {
  auto params = std::make_shared<SetterParams>();
  double first_valid_timestamp = 1.0;
  params->SetReal("BehaviorModel::FirstValidTimestamp", first_valid_timestamp);

  BehaviorMobilRuleBased model(params);
  ASSERT_EQ(model.GetBehaviorStatus(), BehaviorStatus::NOT_STARTED_YET);

  auto observed_world =
      bark::world::tests::make_test_observed_world(0, 0, 0, 0);

  // Return all
  auto traj = model.Plan(first_valid_timestamp / 2, observed_world);
  ASSERT_EQ(model.GetBehaviorStatus(), BehaviorStatus::NOT_STARTED_YET);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
