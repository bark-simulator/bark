// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "gtest/gtest.h"

#include "bark/world/evaluation/ltl/label_functions/agent_near_label_function.hpp"
#include "bark/world/evaluation/ltl/label_functions/lane_change_label_function.hpp"
#include "bark/world/evaluation/ltl/label_functions/left_of_label_function.hpp"
#include "bark/world/evaluation/ltl/label_functions/rel_speed_label_function.hpp"
#include "bark/world/evaluation/ltl/label_functions/right_of_label_function.hpp"
#include "bark/world/evaluation/ltl/label_functions/safe_distance_label_function.hpp"

#include "bark/commons/params/setter_params.hpp"
#include "bark/models/behavior/motion_primitives/macro_actions.hpp"
#include "bark/models/behavior/motion_primitives/primitives/primitive_const_acc_change_to_left.hpp"
#include "bark/models/behavior/motion_primitives/primitives/primitive_const_acc_change_to_right.hpp"
#include "bark/world/tests/make_test_world.hpp"

using namespace bark::world::evaluation;
using namespace bark::world::tests;
using namespace bark::models::behavior;
using namespace bark::models::behavior::primitives;

using bark::commons::SetterParams;

TEST(label_test, right_of) {
  auto evaluator = LabelFunctionPtr(new RightOfLabelFunction("r_v"));
  auto world = MakeTestWorldHighway();
  auto observed_worlds = world->Observe({1, 4});
  auto labels1 = evaluator->Evaluate(observed_worlds[0]);
  Label label1("r_v", 4);
  EXPECT_FALSE(labels1[label1]);

  auto labels2 = evaluator->Evaluate(observed_worlds[1]);
  Label label2("r_v", 1);
  EXPECT_TRUE(labels2[label2]);
}

TEST(label_test, left_of) {
  auto evaluator = LabelFunctionPtr(new LeftOfLabelFunction("l_v"));
  auto world = MakeTestWorldHighway();
  auto observed_worlds = world->Observe({1, 4});
  auto labels1 = evaluator->Evaluate(observed_worlds[0]);
  Label label1("l_v", 4);
  EXPECT_TRUE(labels1[label1]);

  auto labels2 = evaluator->Evaluate(observed_worlds[1]);
  Label label2("l_v", 1);
  EXPECT_FALSE(labels2[label2]);
}

TEST(label_test, safe_distance) {
  const int ego_id = 1;
  double v_0 = 8.0;
  double dv = 0.0;
  double delta = 1.0;
  double a_e = -8.0;
  double a_o = -8.0;

  auto evaluator = LabelFunctionPtr(
      new SafeDistanceLabelFunction("safe_distance", false, delta, a_e, a_o));
  auto label = evaluator->GetLabel();
  double stop_dist = v_0 * delta + v_0 * v_0 / (2.0 * -a_e);

  // Case 1
  assert(stop_dist > 1.0);
  auto world = make_test_world(1, stop_dist - 1.0, v_0, dv);
  auto observed_world = world->Observe({ego_id})[0];
  auto labels = evaluator->Evaluate(observed_world);
  EXPECT_TRUE(labels[label]);

  // Case 2
  double dist = 5.0;
  auto world2 = make_test_world(1, dist, v_0, dv);
  auto observed_world2 = world2->Observe({ego_id})[0];
  auto labels2 = evaluator->Evaluate(observed_world2);
  EXPECT_FALSE(labels2[label]);

  // Case 3
  dist = 2.0;
  auto world3 = make_test_world(1, dist, v_0, dv);
  auto observed_world3 = world3->Observe({ego_id})[0];
  auto labels3 = evaluator->Evaluate(observed_world3);
  EXPECT_FALSE(labels3[label]);

  // Case 4
  delta = 0.5;
  dist = 4.5;
  evaluator = LabelFunctionPtr(
      new SafeDistanceLabelFunction("safe_distance", false, delta, a_e, a_o));
  auto world4 = make_test_world(1, dist, v_0, dv);
  auto observed_world4 = world4->Observe({ego_id})[0];
  auto labels4 = evaluator->Evaluate(observed_world4);
  EXPECT_TRUE(labels4[label]);

  // Case 5
  dist = 6.0;
  auto world5 = make_test_world(1, dist, v_0, dv);
  auto observed_world5 = world5->Observe({ego_id})[0];
  auto labels5 = evaluator->Evaluate(observed_world5);
  EXPECT_TRUE(labels5[label]);
}

TEST(label_test, lane_change_right) {
  auto evaluator = LabelFunctionPtr(new LaneChangeLabelFunction("lane_change"));
  auto label = evaluator->GetLabel();
  auto world = MakeTestWorldHighway();
  auto params = std::make_shared<SetterParams>();
  AgentId id = 1;
  auto beh_change_right = std::make_shared<BehaviorMPMacroActions>(params);
  auto motion_idx = beh_change_right->AddMotionPrimitive(
      PrimitivePtr(new PrimitiveConstAccChangeToRight(params)));
  beh_change_right->ActionToBehavior(motion_idx);
  auto agent_1 = world->GetAgent(id);
  agent_1->SetBehaviorModel(beh_change_right);
  auto current_pos = agent_1->GetCurrentPosition();
  const auto right_lc =
      agent_1->GetRoadCorridor()->GetLeftRightLaneCorridor(current_pos).second;
  auto current_lc =
      agent_1->GetRoadCorridor()->GetCurrentLaneCorridor(current_pos);
  auto observed_world = world->Observe({id})[0];
  auto labels = evaluator->Evaluate(observed_world);
  ASSERT_FALSE(labels[label]);
  while (right_lc != current_lc) {
    observed_world = world->Observe({id})[0];
    auto labels = evaluator->Evaluate(observed_world);
    ASSERT_FALSE(labels[label]);
    world->Step(0.5);
    agent_1 = world->GetAgent(id);
    current_pos = agent_1->GetCurrentPosition();
    current_lc =
        agent_1->GetRoadCorridor()->GetCurrentLaneCorridor(current_pos);
  }
  observed_world = world->Observe({id})[0];
  labels = evaluator->Evaluate(observed_world);
  EXPECT_TRUE(labels[label]);
}

TEST(label_test, lane_change_left) {
  auto evaluator = LabelFunctionPtr(new LaneChangeLabelFunction("lane_change"));
  auto label = evaluator->GetLabel();
  auto world = MakeTestWorldHighway();
  auto params = std::make_shared<SetterParams>();
  AgentId id = 4;
  auto beh_change_left = std::make_shared<BehaviorMPMacroActions>(params);
  auto motion_idx = beh_change_left->AddMotionPrimitive(
      PrimitivePtr(new PrimitiveConstAccChangeToLeft(params)));
  beh_change_left->ActionToBehavior(motion_idx);
  auto agent_4 = world->GetAgent(id);
  agent_4->SetBehaviorModel(beh_change_left);
  auto current_pos = agent_4->GetCurrentPosition();
  const auto left_lc =
      agent_4->GetRoadCorridor()->GetLeftRightLaneCorridor(current_pos).first;
  auto current_lc =
      agent_4->GetRoadCorridor()->GetCurrentLaneCorridor(current_pos);
  auto observed_world = world->Observe({id})[0];
  auto labels = evaluator->Evaluate(observed_world);
  ASSERT_FALSE(labels[label]);
  while (left_lc != current_lc) {
    observed_world = world->Observe({id})[0];
    auto labels = evaluator->Evaluate(observed_world);
    ASSERT_FALSE(labels[label]);
    world->Step(0.5);
    agent_4 = world->GetAgent(id);
    current_pos = agent_4->GetCurrentPosition();
    current_lc =
        agent_4->GetRoadCorridor()->GetCurrentLaneCorridor(current_pos);
  }
  observed_world = world->Observe({id})[0];
  labels = evaluator->Evaluate(observed_world);
  EXPECT_TRUE(labels[label]);
}

TEST(label_test, rel_speed_gt) {
  auto evaluator =
      LabelFunctionPtr(new RelSpeedLabelFunction("rel_speed_gt", 10.0));
  auto world = make_test_world(1, 20.0, 20.0, 15.0);
  auto observed_worlds = world->Observe({1});
  auto labels1 = evaluator->Evaluate(observed_worlds[0]);
  auto label1 = evaluator->GetLabel(2);
  EXPECT_TRUE(labels1[label1]);

  auto world2 = make_test_world(1, 20.0, 20.0, 5.0);
  auto observed_worlds2 = world2->Observe({1});
  auto labels2 = evaluator->Evaluate(observed_worlds2[0]);
  auto label2 = evaluator->GetLabel(2);
  EXPECT_FALSE(labels2[label2]);
}

TEST(label_test, agent_near) {
  auto evaluator = LabelFunctionPtr(new AgentNearLabelFunction("near", 10.0));
  auto world = make_test_world(1, 5.0, 20.0, 0.0);
  auto observed_worlds = world->Observe({1});
  auto labels1 = evaluator->Evaluate(observed_worlds[0]);
  auto label1 = evaluator->GetLabel(2);
  EXPECT_TRUE(labels1[label1]);

  auto world2 = make_test_world(1, 15.0, 20.0, 0.0);
  auto observed_worlds2 = world2->Observe({1});
  auto labels2 = evaluator->Evaluate(observed_worlds2[0]);
  auto label2 = evaluator->GetLabel(2);
  EXPECT_FALSE(labels2[label2]);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}