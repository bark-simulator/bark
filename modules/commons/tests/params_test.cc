// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include <iostream>
#include <fstream>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "gtest/gtest.h"

#include "modules/commons/params/params.hpp"


// TODO(fortiss): fill our this test
TEST(load_test, param_tests) {
  std::cout << "Start test\n";

  // Params my_param_server = Params("Root");
  // my_param_server.load("/home/meissner/fortiss-behave/modules/commons/Params/example.json");
  // cout << my_param_server.getString("uno","unodefault","") << "\n";
  // cout << my_param_server.getString("monkey","monkeydefault","") << "\n";
  // my_param_server.dump("/home/meissner/fortiss-behave/modules/commons/Params/example_rec.json");
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
