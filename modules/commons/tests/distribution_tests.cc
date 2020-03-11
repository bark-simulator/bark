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
#include "modules/commons/distribution/distributions_1d.hpp"


// TODO(fortiss): fill our this test
TEST(distribution_test, normal_dist_1d) {

  auto params_ptr = std::make_shared<modules::commons::SetterParams>(true);
  params_ptr->SetReal("Mean", -3.0f);
  params_ptr->SetReal("StdDev", 2.0f);

  auto dist_normal = modules::commons::NormalDistribution1D(params_ptr);

  size_t samples = 10000;
  double mean = 0.0f;
  for(size_t i = 0; i< samples; ++i) {
    mean += dist_normal.Sample();
  }
  mean /= samples;
  EXPECT_NEAR(mean, -3.0f, 0.01);

  double std_dev = 0.0f;
  for (size_t i = 0; i < samples; ++i) {
    auto sample = dist_normal.Sample();
    std_dev += abs(mean - sample) * abs(mean - sample);
  }
  EXPECT_NEAR(sqrt(std_dev/samples), 2.0f, 0.01);


  
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
