// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "boost/variant.hpp"
#include "gtest/gtest.h"

#include "bark/models/behavior/safety_behavior/safety_behavior.hpp"
#include "bark/models/behavior/rss_behavior/rss_behavior.hpp"
#include "bark/world/observed_world.hpp"
#include "bark/world/tests/make_test_world.hpp"

using bark::models::behavior::BehaviorSafety;
using bark::models::behavior::RSSBehavior;
using bark::models::behavior::BehaviorModelPtr;
using bark::models::behavior::BehaviorStatus;
using bark::world::Agent;
using bark::world::ObservedWorld;
using bark::world::World;
using bark::world::WorldPtr;

TEST(safe_behavior, init) {
  auto safety_behavior = BehaviorSafety(nullptr);
}


TEST(rss_behavior, init) {
  auto rss_behavior = RSSBehavior(nullptr);

}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}