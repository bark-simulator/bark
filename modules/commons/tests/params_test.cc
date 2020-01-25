// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include <iostream>
#include <fstream>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "gtest/gtest.h"

#include "modules/commons/params/setter_params.hpp"


// TODO(fortiss): fill our this test
TEST(setter_params, param_tests) {
  std::cout << "Start test\n";

  modules::commons::SetterParams params;

  params.SetReal("Test::2", 0.5f);
  EXPECT_EQ(params.GetReal("Test::2","", 1.0f), 0.5f);

  params.SetInt("Test::2", 2);
  EXPECT_EQ(params.GetInt("Test::2", "", 1),2);

  params.SetBool("Test::5", true);
  EXPECT_EQ(params.GetBool("Test::5", "", false), true);

  params.SetListListFloat("Test::2", {{0,1}, {0,2},{0.5,1.5}});
  EXPECT_EQ(params.GetListListFloat("Test::2", "", {{0,1}, {0,2}}), std::vector<std::vector<float>>({{0,1}, {0,2},{0.5,1.5}}));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
