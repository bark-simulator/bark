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
#include <glog/logging.h>

#include "bark/commons/distribution/distributions_1d.hpp"
#include "bark/commons/distribution/multivariate_normal.hpp"
#include "bark/commons/params/setter_params.hpp"
#include "bark/commons/distribution/distribution.hpp"

// TODO(fortiss): fill our this test
TEST(distribution_test, normal_dist_1d) {
  auto params_ptr = std::make_shared<bark::commons::SetterParams>(true);
  params_ptr->SetReal("Mean", -3.0);
  params_ptr->SetReal("StdDev", 2.0);
  params_ptr->SetInt("RandomSeed", 1000.0);

  auto dist_normal = bark::commons::NormalDistribution1D(params_ptr);

  size_t samples = 100000;
  double mean = 0.0;
  for (size_t i = 0; i < samples; ++i) {
    mean += dist_normal.Sample()[0];
  }
  mean /= samples;
  EXPECT_NEAR(mean, -3.0, 0.01);

  double std_dev = 0.0;
  for (size_t i = 0; i < samples; ++i) {
    auto sample = dist_normal.Sample()[0];
    std_dev += abs(mean - sample) * abs(mean - sample);
  }
  EXPECT_NEAR(sqrt(std_dev / samples), 2.0, 0.01);
}

// TODO(fortiss): fill out this test
TEST(distribution_test, uniform_dist_1d) {
  auto params_ptr = std::make_shared<bark::commons::SetterParams>(true);
  const double lower_bound = -3.0;
  const double upper_bound = 10.0;
  params_ptr->SetReal("LowerBound", -3.0);
  params_ptr->SetReal("UpperBound", 10.0);
  params_ptr->SetInt("RandomSeed", 1000.0);

  auto dist_uniform = bark::commons::UniformDistribution1D(params_ptr);

  auto support = dist_uniform.GetSupport();
  EXPECT_NEAR(support[0].first, -3.0, 0.0001f);
  EXPECT_NEAR(support[0].second, 10.0, 0.0001f);

  size_t samples = 10000;
  double mean = 0.0;
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

  auto uniform_prob = 1.0 / (upper_bound - lower_bound);
  for (const auto& container : sample_container) {
    auto bucket_prob =
        static_cast<double>(container.size()) / static_cast<double>(samples);
    EXPECT_NEAR(bucket_prob, bucket_size * uniform_prob, 0.01);
  }

  EXPECT_NEAR(mean / samples, (lower_bound + upper_bound) / 2.0, 0.05);

  EXPECT_NEAR(dist_uniform.Density({2.0}), uniform_prob, 0.001f);
  EXPECT_NEAR(dist_uniform.Density({3.0}), uniform_prob, 0.001f);
  EXPECT_NEAR(dist_uniform.Density({12.0}), 0.0, 0.001f);
  EXPECT_NEAR(dist_uniform.Density({-4.0}), 0.0, 0.001f);

  EXPECT_NEAR(dist_uniform.CDF({0.0}), 3.0 * uniform_prob, 0.001f);
}

// TODO(fortiss): fill our this test
TEST(distribution_test, bernoulli_dist_1d) {
  auto params_ptr = std::make_shared<bark::commons::SetterParams>(true);
  params_ptr->SetReal("Probability", 0.3);
  params_ptr->SetInt("RandomSeed", 1000.0);

  auto dist_bernoulli = bark::commons::BernoulliDistribution1D(params_ptr);

  size_t samples = 100000;
  double mean = 0.0;
  for (size_t i = 0; i < samples; ++i) {
    mean += dist_bernoulli.Sample()[0];
  }
  mean /= samples;
  EXPECT_NEAR(mean, 0.3, 0.01);
}

TEST(distribution_test, multivariate_distribution) {
  // First test zero covariances
  auto params_ptr1 = std::make_shared<bark::commons::SetterParams>(true);
  //const double lower_bound = -3.0;
  //const double upper_bound = 10.0;
  params_ptr1->SetListListFloat(
      "Covariance", {{1.0, 0.2, 0.1}, {0.2, 3.0, -0.5}, {0.1, -0.5, 0.125553}});
  params_ptr1->SetListFloat("Mean", {1.2, 12.0, 0.1234f});
  params_ptr1->SetInt("RandomSeed", 1000.0);
  auto dist_multivariate = bark::commons::MultivariateDistribution(params_ptr1);

  size_t samples = 500000;
  std::vector<double> mean(3, 0.0);
  for (size_t i = 0; i < samples; ++i) {
    auto sample = dist_multivariate.Sample();
    for (int j = 0; j < 3; ++j) {
      mean[j] += sample[j];
    }
  }
  for (int j = 0; j < 3; ++j) {
    mean[j] /= samples;
  }
  auto mean_desired = params_ptr1->GetListFloat("Mean", "", {});
  for (int j = 0; j < 3; ++j) {
    EXPECT_NEAR(mean[j], mean_desired[j], 0.01);
  }

  std::vector<std::vector<double>> covar(3, std::vector<double>(3, 0.0));
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
  auto covar_desired = params_ptr1->GetListListFloat("Covariance", "", {{}});
  for (int j = 0; j < 3; ++j) {
    for (int z = 0; z < 3; ++z) {
      EXPECT_NEAR(covar[j][z], covar_desired[j][z], 0.01);
    }
  }

  // Test density function
  auto params_ptr2 = std::make_shared<bark::commons::SetterParams>(true);
  params_ptr2->SetListListFloat("Covariance", {{4.0, 1.3}, {1.3, 1.0}});
  params_ptr2->SetListFloat("Mean", {3.1, 2.0});
  params_ptr2->SetInt("RandomSeed", 1000.0);
  auto dist_multivariate2d = bark::commons::MultivariateDistribution(params_ptr2);

  auto test_p1 = bark::commons::RandomVariate{3.1, 2.};
  auto test_p2 = bark::commons::RandomVariate{4.,2.6};
  auto test_p3 = bark::commons::RandomVariate{3.4, 1.58};

  auto p1_likelihood = dist_multivariate2d.Density(test_p1);
  auto p2_likelihood = dist_multivariate2d.Density(test_p2);
  auto p3_likelihood = dist_multivariate2d.Density(test_p3);

  ASSERT_FLOAT_EQ(p1_likelihood, 0.10471626);
  ASSERT_FLOAT_EQ(p2_likelihood, 0.08719418);
  ASSERT_FLOAT_EQ(p3_likelihood, 0.08211638);
}

TEST(distribution_test, change_seed) {
  auto params_ptr = std::make_shared<bark::commons::SetterParams>(true);
  params_ptr->SetReal("Mean", -3.0);
  params_ptr->SetReal("StdDev", 2.0);
  params_ptr->SetInt("RandomSeed", 1000.0);

  auto dist_normal1 = bark::commons::NormalDistribution1D(params_ptr);
  auto dist_normal2 = bark::commons::NormalDistribution1D(params_ptr);

  auto sample1 = dist_normal1.Sample()[0];
  auto sample2 = dist_normal2.Sample()[0];
  EXPECT_EQ(sample1, sample2);

  dist_normal2.ChangeSeed(2000);
  sample1 = dist_normal1.Sample()[0];
  sample2 = dist_normal2.Sample()[0];
  EXPECT_TRUE(sample1 != sample2);
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
