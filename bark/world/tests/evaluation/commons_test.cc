// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "gtest/gtest.h"

#include "bark/world/evaluation/commons.hpp"
#include "bark/commons/params/setter_params.hpp"
#include "bark/world/tests/make_test_world.hpp"

using namespace bark::world::evaluation;
using namespace bark::world::tests;
using bark::commons::SetterParams;

TEST(capture_test, base_functionality) {
  auto world = make_test_world(1, 5.0, 20.0, 0.0);
  using bark::world::evaluation::CaptureAgentStates;
  auto captured_states = CaptureAgentStates(*world);
  //   eval_res["capture_states"]);
  EXPECT_EQ(captured_states.size(), 2);
  EXPECT_EQ(captured_states["state_1"], world->GetAgents()[1]->GetCurrentState());
  EXPECT_EQ(captured_states["state_2"], world->GetAgents()[2]->GetCurrentState());

}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}