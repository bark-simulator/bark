// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <fstream>
#include <iostream>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include "gtest/gtest.h"

#include "bark/commons/util/util.hpp"

// TODO(@all): fill our this test
TEST(load_test, param_tests) {
  bool temp = false;
  BARK_EXPECT_TRUE(temp);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
