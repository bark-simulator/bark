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
#include "bark/world/evaluation/base_evaluator.hpp"
#include "bark/world/tests/make_test_world.hpp"

using bark::models::behavior::BehaviorSafety;
using bark::models::behavior::BehaviorIDMLaneTracking;
using bark::models::behavior::BehaviorIDMClassic;
using bark::models::behavior::BehaviorRSSConformant;
using bark::models::behavior::BehaviorModelPtr;
using bark::models::behavior::BehaviorStatus;
using bark::world::Agent;
using bark::world::evaluation::EvaluationReturn;
using bark::world::evaluation::BaseEvaluator;
using bark::world::ObservedWorld;
using bark::world::World;
using bark::world::WorldPtr;
using bark::commons::SetterParams;


class DummyRSSEvaluator : public BaseEvaluator {
 public:
  DummyRSSEvaluator(int step_trigger) :
    step_count_(0), step_trigger_(step_trigger) {}
  virtual EvaluationReturn Evaluate(const World& world) {
    step_count_++;
    return (step_count_ > step_trigger_);
  }
  virtual EvaluationReturn Evaluate(
    const ObservedWorld& observed_world) {
    step_count_++;
    return (step_count_ > step_trigger_);
  }

 private:
  int step_count_;
  int step_trigger_;
};

TEST(safe_behavior, init) {
  auto params = std::make_shared<SetterParams>();
  auto behavior_lane_tracking = std::make_shared<BehaviorIDMLaneTracking>(params);
  auto safety_behavior = BehaviorSafety(params);
  safety_behavior.SetBehaviorModel(behavior_lane_tracking);
}


TEST(rss_behavior, init) {
  // safety behavior
  auto params = std::make_shared<SetterParams>();
  auto behavior_lane_tracking = std::make_shared<BehaviorIDMLaneTracking>(
    params);
  std::shared_ptr<BehaviorSafety> safety_behavior =
    std::make_shared<BehaviorSafety>(params);
  safety_behavior->SetBehaviorModel(behavior_lane_tracking);

  // rss behavior
  auto rss_behavior = BehaviorRSSConformant(params);
  auto behavior_idm_classic = std::make_shared<BehaviorIDMClassic>(params);
  rss_behavior.SetNominalBehaviorModel(behavior_idm_classic);
  rss_behavior.SetSafetyBehaviorModel(safety_behavior);

  // set (dummy) RSS evaluator
  std::shared_ptr<BaseEvaluator> rss_eval =
    std::make_shared<DummyRSSEvaluator>(4);
  rss_behavior.SetEvaluator(rss_eval);

  auto behavior_safety_model = rss_behavior.GetBehaviorSafetyModel();
  auto safety_params = behavior_safety_model->GetBehaviorSafetyParams();

  // check whether it works to set the velocity
  // NOTE: this will only work once the Plan() function of the BehaviorRSS has been called
  // EXPECT_EQ(
  //   safety_params->GetReal("BehaviorIDMClassic::DesiredVelocity", "", -1.), 0.);

  // TODO: add lane corridor assertion

  // TODO: how do we test the full model
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}