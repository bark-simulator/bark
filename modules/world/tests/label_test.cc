// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "gtest/gtest.h"

#include "modules/world/evaluation/labels/left_of_label_function.hpp"
#include "modules/world/evaluation/labels/right_of_label_function.hpp"
#include "modules/world/evaluation/labels/safe_distance_label_function.hpp"
#include "modules/world/tests/make_test_world.hpp"

using namespace modules::world::evaluation;
using namespace modules::world::tests;

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

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}