// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "gtest/gtest.h"

#include "modules/world/evaluation/labels/left_of_label_evaluator.hpp"
#include "modules/world/evaluation/labels/right_of_label_evaluator.hpp"
#include "modules/world/tests/make_test_world.hpp"

using namespace modules::world::evaluation;
using namespace modules::world::tests;

TEST(label_test, right_of) {
  auto evaluator = LabelEvaluatorPtr(new RightOfLabelEvaluator("r_v"));
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
  auto evaluator = LabelEvaluatorPtr(new LeftOfLabelEvaluator("l_v"));
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

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}