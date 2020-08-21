// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "gtest/gtest.h"

#include "bark/world/evaluation/evaluator_capture_states.hpp"
#include "bark/commons/params/setter_params.hpp"
#include "bark/world/tests/make_test_world.hpp"

using namespace bark::world::evaluation;
using namespace bark::world::tests;
using bark::commons::SetterParams;

TEST(capture_test, base_functionality) {
  auto world = make_test_world(1, 5.0, 20.0, 0.0);
  // auto observed_worlds = world->Observe({1});
  auto eval_capture_states = std::make_shared<EvaluatorCaptureAgentStates>();
  world->AddEvaluator("capture_states", eval_capture_states);
  auto eval_res = world->Evaluate();

  auto casted_res = boost::get<std::map<std::string, State>>(
    eval_res["capture_states"]);
  EXPECT_EQ(casted_res.size(), 2);
  EXPECT_EQ(casted_res["state_1"], world->GetAgents()[1]->GetCurrentState());
  EXPECT_EQ(casted_res["state_2"], world->GetAgents()[2]->GetCurrentState());
  for (const auto& res : casted_res) {
    std::cout << res.first << ": \n" << res.second << std::endl;
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}