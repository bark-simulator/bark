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
using bark::world::tests::make_test_world;


class DummyRSSEvaluator : public BaseEvaluator {
 public:
  DummyRSSEvaluator(int step_trigger) :
    step_count_(0), step_trigger_(step_trigger) {}
  virtual EvaluationReturn Evaluate(const World& world) {
    step_count_++;
    return (step_count_ >= step_trigger_);
  }
  virtual EvaluationReturn Evaluate(
    const ObservedWorld& observed_world) {
    step_count_++;
    return (step_count_ >= step_trigger_);
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
}


void FwSim(int steps, const WorldPtr& world, double dt = 0.2) {
  for (int i=0; i < steps; i++) {
    world->Step(dt);
  }
}

TEST(rss_behavior, rss_behavior_system_test) {
  // Test-strategy: both time the ego agent is controlled by the
  // BehaviorIDMLaneTracking and attempts to change from the right to the
  // left lane (by setting the target corridor). One time the dummy rss
  // evaluator intervenes and the lane-change is aborted.
  auto params = std::make_shared<SetterParams>();

  // First case, we start with the desired velocity. After num steps, we should
  // advance
  float ego_velocity, rel_distance = 7.0, velocity_difference = 0.0;
  float time_step = 0.2f;

  // should place an agent on the right lane (+3.5)
  WorldPtr world =
    make_test_world(0, rel_distance, ego_velocity, velocity_difference);
  auto ego_agent = world->GetAgents().begin()->second;

  // nominal BehaviorIDMLaneTracking (changes to the left)
  auto nominal_behavior_lane_tracking =
    std::make_shared<BehaviorIDMLaneTracking>(params);
  auto road_corridor = ego_agent->GetRoadCorridor();
  auto lane_corridors = road_corridor->GetUniqueLaneCorridors();
  nominal_behavior_lane_tracking->SetConstantLaneCorridor(lane_corridors[0]);

  // safety behavior
  std::shared_ptr<BehaviorSafety> safety_behavior =
    std::make_shared<BehaviorSafety>(params);
  
  // rss behavior
  std::shared_ptr<BehaviorRSSConformant> rss_behavior =
    std::make_shared<BehaviorRSSConformant>(params);
  rss_behavior->SetNominalBehaviorModel(nominal_behavior_lane_tracking);
  rss_behavior->SetSafetyBehaviorModel(safety_behavior);

  // set behavior model
  ego_agent->SetBehaviorModel(rss_behavior);
  std::shared_ptr<BaseEvaluator> rss_eval_do_not_trigger =
    std::make_shared<DummyRSSEvaluator>(1000);
  rss_behavior->SetEvaluator(rss_eval_do_not_trigger);
  auto world_nominal = world;
  auto world_rss_triggered = world->Clone();

  // simulate nominal
  FwSim(20, world_nominal);
  // TODO: assert lane corridor
  std::cout << ego_agent->GetCurrentState() << std::endl;

  // simulate triggered
  auto ego_agent_triggered = world_rss_triggered->GetAgents().begin()->second;
  std::shared_ptr<BaseEvaluator> rss_eval_trigger =
    std::make_shared<DummyRSSEvaluator>(5);
  rss_behavior->SetEvaluator(rss_eval_trigger);
  ego_agent_triggered->SetBehaviorModel(rss_behavior);
  FwSim(20, world_rss_triggered);
  // TODO: assert lane corridor
  std::cout << ego_agent_triggered->GetCurrentState() << std::endl;

}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}