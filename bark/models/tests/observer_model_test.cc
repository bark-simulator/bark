// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/models/observer/observer_model.hpp"
#include "gtest/gtest.h"
#include "bark/commons/params/setter_params.hpp"

using bark::commons::SetterParams;

TEST(observer_model, IF_tests) {
  auto params = std::make_shared<SetterParams>();


}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
