// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <math.h>
#include <fstream>
#include <iostream>

#include "gtest/gtest.h"

#include "bark/commons/math/vector.hpp"

TEST(math_test, mean) {
  std::vector<double> v{1, 2, 3};
  EXPECT_EQ(bark::commons::math::CalculateMean(v), 2);
}

TEST(math_test, mean_with_nan) {
  std::vector<double> v{1, 2, 3};
  double val = nan("");
  v.push_back(val);
  EXPECT_TRUE(isnan(bark::commons::math::CalculateMean(v)));
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
