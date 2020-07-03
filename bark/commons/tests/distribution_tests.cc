// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <fstream>
#include <iostream>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include "gtest/gtest.h"

#include "bark/commons/distribution/distributions_1d.hpp"
#include "bark/commons/distribution/multivariate_normal.hpp"
#include "bark/commons/params/setter_params.hpp"

// TODO(fortiss): fill our this test
TEST(distribution_test, normal_dist_1d) {
  auto params_ptr = std::make_shared<bark::commons::SetterParams>(true);
  params_ptr->SetReal("Mean", -3.0f);
  params_ptr->SetReal("StdDev", 2.0f);
  params_ptr->SetInt("RandomSeed", 1000.0f);

  auto dist_normal = bark::commons::NormalDistribution1D(params_ptr);

  size_t samples = 30000;
  double mean = 0.0f;
  for (size_t i = 0; i < samples; ++i) {
    mean += dist_normal.Sample()[0];
  }
  mean /= samples;
  EXPECT_NEAR(mean, -3.0f, 0.01);

  double std_dev = 0.0f;
  for (size_t i = 0; i < samples; ++i) {
    auto sample = dist_normal.Sample()[0];
    std_dev += abs(mean - sample) * abs(mean - sample);
  }
  EXPECT_NEAR(sqrt(std_dev / samples), 2.0f, 0.01);
}

// TODO(fortiss): fill our this test
TEST(distribution_test, uniform_dist_1d) {
  auto params_ptr = std::make_shared<bark::commons::SetterParams>(true);
  const double lower_bound = -3.0f;
  const double upper_bound = 10.0f;
  params_ptr->SetReal("LowerBound", -3.0f);
  params_ptr->SetReal("UpperBound", 10.0f);
  params_ptr->SetInt("RandomSeed", 1000.0f);

  auto dist_uniform = bark::commons::UniformDistribution1D(params_ptr);

  size_t samples = 10000;
  double mean = 0.0f;
  int num_buckets = 100;
  double bucket_size = (upper_bound - lower_bound) / num_buckets;
  std::vector<std::vector<double>> sample_container(num_buckets);
  for (size_t i = 0; i < samples; ++i) {
    auto sample = dist_uniform.Sample()[0];
    EXPECT_TRUE(sample <= upper_bound);
    EXPECT_TRUE(sample >= lower_bound);
    sample_container[std::floor((sample - lower_bound) / bucket_size)]
        .push_back(sample);
    mean += sample;
  }

  auto uniform_prob = 1.0f / (upper_bound - lower_bound);
  for (const auto& container : sample_container) {
    auto bucket_prob =
        static_cast<float>(container.size()) / static_cast<float>(samples);
    EXPECT_NEAR(bucket_prob, bucket_size * uniform_prob, 0.01);
  }

  EXPECT_NEAR(mean / samples, (lower_bound + upper_bound) / 2.0f, 0.05);

  EXPECT_NEAR(dist_uniform.Density({2.0f}), uniform_prob, 0.001f);
  EXPECT_NEAR(dist_uniform.Density({3.0f}), uniform_prob, 0.001f);
  EXPECT_NEAR(dist_uniform.Density({12.0f}), 0.0f, 0.001f);
  EXPECT_NEAR(dist_uniform.Density({-4.0f}), 0.0f, 0.001f);

  EXPECT_NEAR(dist_uniform.CDF({0.0f}), 3.0f * uniform_prob, 0.001f);
}

TEST(distribution_test, multivariate_distribution) {
  // First test zero covariances
  auto params_ptr = std::make_shared<bark::commons::SetterParams>(true);
  const double lower_bound = -3.0f;
  const double upper_bound = 10.0f;
  params_ptr->SetListListFloat(
      "Covariance", {{1.0, 0.2, 0.1}, {0.2, 3.0, -0.5}, {0.1, -0.5, 0.125553}});
  params_ptr->SetListFloat("Mean", {1.2f, 12.0f, 0.1234f});
  params_ptr->SetInt("RandomSeed", 1000.0f);

  auto dist_multivariate = bark::commons::MultivariateDistribution(params_ptr);

  size_t samples = 200000;
  std::vector<double> mean(3, 0.0f);
  for (size_t i = 0; i < samples; ++i) {
    auto sample = dist_multivariate.Sample();
    for (int j = 0; j < 3; ++j) {
      mean[j] += sample[j];
    }
  }
  for (int j = 0; j < 3; ++j) {
    mean[j] /= samples;
  }
  auto mean_desired = params_ptr->GetListFloat("Mean", "", {});
  for (int j = 0; j < 3; ++j) {
    EXPECT_NEAR(mean[j], mean_desired[j], 0.01);
  }

  std::vector<std::vector<double>> covar(3, std::vector<double>(3, 0.0f));
  for (size_t i = 0; i < samples; ++i) {
    auto sample = dist_multivariate.Sample();
    for (int j = 0; j < 3; ++j) {
      for (int z = 0; z < 3; ++z) {
        covar[j][z] += (sample[j] - mean[j]) * (sample[z] - mean[z]);
      }
    }
  }
  for (int j = 0; j < 3; ++j) {
    for (int z = 0; z < 3; ++z) {
      covar[j][z] /= samples;
    }
  }
  auto covar_desired = params_ptr->GetListListFloat("Covariance", "", {{}});
  for (int j = 0; j < 3; ++j) {
    for (int z = 0; z < 3; ++z) {
      EXPECT_NEAR(covar[j][z], covar_desired[j][z], 0.01);
    }
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
