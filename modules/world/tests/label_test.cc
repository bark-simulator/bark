// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "gtest/gtest.h"

#include "modules/commons/params/default_params.hpp"
#include "modules/models/behavior/motion_primitives/macro_actions.hpp"
#include "modules/models/behavior/motion_primitives/primitives/primitive_const_acc_change_to_left.hpp"
#include "modules/models/behavior/motion_primitives/primitives/primitive_const_acc_change_to_right.hpp"
#include "modules/world/evaluation/labels/agent_near_label_function.hpp"
#include "modules/world/evaluation/labels/lane_change_label_function.hpp"
#include "modules/world/evaluation/labels/left_of_label_function.hpp"
#include "modules/world/evaluation/labels/rel_speed_label_function.hpp"
#include "modules/world/evaluation/labels/right_of_label_function.hpp"
#include "modules/world/evaluation/labels/safe_distance_label_function.hpp"
#include "modules/world/tests/make_test_world.hpp"

using namespace modules::world::evaluation;
using namespace modules::world::tests;
using namespace modules::models::behavior;
using namespace modules::models::behavior::primitives;

using modules::commons::DefaultParams;

TEST(label_test, right_of) {
  auto evaluator = LabelFunctionPtr(new RightOfLabelFunction("r_v"));
  auto world = MakeTestWorldHighway();
  world->AddLabels({evaluator});
  auto observed_worlds = world->Observe({1, 4});
  auto labels1 = observed_worlds[0].EvaluateLabels();
  Label label1("r_v", 4);
  EXPECT_FALSE(labels1[label1]);

  auto labels2 = observed_worlds[1].EvaluateLabels();
  Label label2("r_v", 1);
  EXPECT_TRUE(labels2[label2]);
}

TEST(label_test, left_of) {
  auto evaluator = LabelFunctionPtr(new LeftOfLabelFunction("l_v"));
  auto world = MakeTestWorldHighway();
  world->AddLabels({evaluator});
  auto observed_worlds = world->Observe({1, 4});
  auto labels1 = observed_worlds[0].EvaluateLabels();
  Label label1("l_v", 4);
  EXPECT_TRUE(labels1[label1]);

  auto labels2 = observed_worlds[1].EvaluateLabels();
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
  world->AddLabels({evaluator});
  auto observed_world = world->Observe({ego_id})[0];
  auto labels = observed_world.EvaluateLabels();
  EXPECT_TRUE(labels[label]);

  // Case 2
  double dist = 5.0;
  auto world2 = make_test_world(1, dist, v_0, dv);
  world2->AddLabels({evaluator});
  auto observed_world2 = world2->Observe({ego_id})[0];
  auto labels2 = observed_world2.EvaluateLabels();
  EXPECT_FALSE(labels2[label]);

  // Case 3
  dist = 2.0;
  auto world3 = make_test_world(1, dist, v_0, dv);
  world3->AddLabels({evaluator});
  auto observed_world3 = world3->Observe({ego_id})[0];
  auto labels3 = observed_world3.EvaluateLabels();
  EXPECT_FALSE(labels3[label]);

  // Case 4
  delta = 0.5;
  dist = 4.5;
  evaluator = LabelFunctionPtr(
      new SafeDistanceLabelFunction("safe_distance", false, delta, a_e, a_o));
  auto world4 = make_test_world(1, dist, v_0, dv);
  world4->AddLabels({evaluator});
  auto observed_world4 = world4->Observe({ego_id})[0];
  auto labels4 = observed_world4.EvaluateLabels();
  EXPECT_TRUE(labels4[label]);

  // Case 5
  dist = 6.0;
  auto world5 = make_test_world(1, dist, v_0, dv);
  world5->AddLabels({evaluator});
  auto observed_world5 = world5->Observe({ego_id})[0];
  auto labels5 = observed_world5.EvaluateLabels();
  EXPECT_TRUE(labels5[label]);
}

TEST(label_test, lane_change_right) {
  auto evaluator = LabelFunctionPtr(new LaneChangeLabelFunction("lane_change"));
  auto label = evaluator->GetLabel();
  auto world = MakeTestWorldHighway();
  world->AddLabels({evaluator});
  auto params = std::make_shared<DefaultParams>();
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
  auto labels = observed_world.EvaluateLabels();
  ASSERT_FALSE(labels[label]);
  while (right_lc != current_lc) {
    observed_world = world->Observe({id})[0];
    labels = observed_world.EvaluateLabels();
    ASSERT_FALSE(labels[label]);
    world->Step(0.5);
    agent_1 = world->GetAgent(id);
    current_pos = agent_1->GetCurrentPosition();
    current_lc =
        agent_1->GetRoadCorridor()->GetCurrentLaneCorridor(current_pos);
  }
  observed_world = world->Observe({id})[0];
  labels = observed_world.EvaluateLabels();
  EXPECT_TRUE(labels[label]);
}

TEST(label_test, lane_change_left) {
  auto evaluator = LabelFunctionPtr(new LaneChangeLabelFunction("lane_change"));
  auto label = evaluator->GetLabel();
  auto world = MakeTestWorldHighway();
  world->AddLabels({evaluator});
  auto params = std::make_shared<DefaultParams>();
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
  auto labels = observed_world.EvaluateLabels();
  ASSERT_FALSE(labels[label]);
  while (left_lc != current_lc) {
    observed_world = world->Observe({id})[0];
    labels = observed_world.EvaluateLabels();
    ASSERT_FALSE(labels[label]);
    world->Step(0.5);
    agent_4 = world->GetAgent(id);
    current_pos = agent_4->GetCurrentPosition();
    current_lc =
        agent_4->GetRoadCorridor()->GetCurrentLaneCorridor(current_pos);
  }
  observed_world = world->Observe({id})[0];
  labels = observed_world.EvaluateLabels();
  EXPECT_TRUE(labels[label]);
}

TEST(label_test, rel_speed_gt) {
  auto evaluator =
      LabelFunctionPtr(new RelSpeedLabelFunction("rel_speed_gt", 10.0));
  auto world = make_test_world(1, 20.0, 20.0, 15.0);
  world->AddLabels({evaluator});
  auto observed_worlds = world->Observe({1});
  auto labels1 = observed_worlds[0].EvaluateLabels();
  auto label1 = evaluator->GetLabel(2);
  EXPECT_TRUE(labels1[label1]);

  auto world2 = make_test_world(1, 20.0, 20.0, 5.0);
  world2->AddLabels({evaluator});
  auto observed_worlds2 = world2->Observe({1});
  auto labels2 = observed_worlds2[0].EvaluateLabels();
  auto label2 = evaluator->GetLabel(2);
  EXPECT_FALSE(labels2[label2]);
}

TEST(label_test, agent_near) {
  auto evaluator = LabelFunctionPtr(new AgentNearLabelFunction("near", 10.0));
  auto world = make_test_world(1, 5.0, 20.0, 0.0);
  world->AddLabels({evaluator});
  auto observed_worlds = world->Observe({1});
  auto labels1 = observed_worlds[0].EvaluateLabels();
  auto label1 = evaluator->GetLabel(2);
  EXPECT_TRUE(labels1[label1]);

  auto world2 = make_test_world(1, 15.0, 20.0, 0.0);
  world2->AddLabels({evaluator});
  auto observed_worlds2 = world2->Observe({1});
  auto labels2 = observed_worlds2[0].EvaluateLabels();
  auto label2 = evaluator->GetLabel(2);
  EXPECT_FALSE(labels2[label2]);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}