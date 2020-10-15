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
#include "bark/models/behavior/safety_behavior/safety_behavior.hpp"
#include "bark/models/behavior/rss_behavior/rss_behavior.hpp"
#include "bark/models/behavior/idm/idm_lane_tracking.hpp"
#include "bark/models/behavior/idm/idm_classic.hpp"
#include "bark/world/observed_world.hpp"
#include "bark/world/tests/make_test_world.hpp"

using bark::models::behavior::BehaviorSafety;
using bark::models::behavior::BehaviorIDMLaneTracking;
using bark::models::behavior::BehaviorIDMClassic;
using bark::models::behavior::RSSBehavior;
using bark::models::behavior::BehaviorModelPtr;
using bark::models::behavior::BehaviorStatus;
using bark::world::Agent;
using bark::world::ObservedWorld;
using bark::world::World;
using bark::world::WorldPtr;
using bark::commons::SetterParams;


TEST(safe_behavior, init) {
  auto params = std::make_shared<SetterParams>();
  auto behavior_lane_tracking = std::make_shared<BehaviorIDMLaneTracking>(params);
  auto safety_behavior = BehaviorSafety(params);
  safety_behavior.SetBehaviorModel(behavior_lane_tracking);
}


TEST(rss_behavior, init) {
  // safety behavior
  auto params = std::make_shared<SetterParams>();
  auto behavior_lane_tracking = std::make_shared<BehaviorIDMLaneTracking>(params);
  auto safety_behavior = BehaviorSafety(params);
  safety_behavior.SetBehaviorModel(behavior_lane_tracking);

  // rss behavior
  auto rss_behavior = RSSBehavior(params);
  auto behavior_idm_classic = std::make_shared<BehaviorIDMClassic>(params);
  rss_behavior.SetBehaviorModel(behavior_idm_classic);
  rss_behavior.SetSafetyBehaviorModel(behavior_lane_tracking);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}