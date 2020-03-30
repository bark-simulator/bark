// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_COMMONS_DISTRIBUTION_MULTIVARIATE_NORMAL_HPP_
#define MODULES_COMMONS_DISTRIBUTION_MULTIVARIATE_NORMAL_HPP_

#include <random>
#include <boost/random.hpp>
#include <boost/math/distributions.hpp>

#include "modules/commons/distribution/distribution.hpp"
#include "modules/commons/params/params.hpp"


namespace modules {
namespace commons {

// Based on https://stackoverflow.com/questions/6142576/sample-from-multivariate-normal-gaussian-distribution-in-c
class MultivariateDistribution : public Distribution  {
  public:
    MultivariateDistribution(const ParamsPtr& params) : Distribution(params),
             seed_(params->GetInt("RandomSeed", "Specifies seed for mersenne twister engine", 1234)),
             generator_(seed_),
             dist_(),
             mean_(MeanFromParams(params)) {
               auto covariance = CovarFromParams(params);
               Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(covar);
               transform_ = eigenSolver.eigenvectors() * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
             }

    virtual RandomVariate Sample();

    virtual Probability Density(const RandomVariate& variate) const {
          return 0.0f;
          };

    virtual Probability CDF(const RandomVariate& variate) const {
      return 0.0f; 
      }; 

    Eigen::MatrixXd CovarFromParams(const ParamsPtr& params) const;
    Eigen::VectorXd MeanFromParams(const ParamsPtr& params) const;

  private:
    RandomSeed seed_;
    std::mt19937 generator_;
    std::normal_distribution<RandomVariableSupport> dist_;
    Eigen::VectorXd mean_;
    Eigen::MatrixXd transform_;
};


inline RandomVariate MultivariateDistribution::Sample() {
    auto eigen_sample = mean_ + transform_ * Eigen::VectorXd{ mean.size() }.unaryExpr([&](auto x) { return dist_(generator_); });
    RandomVariate sample(eigen_sample.data(), eigen_sample.size());
    return sample;
}

inline Eigen::MatrixXd MultivariateDistribution::CovarFromParams(const ParamsPtr& params) const {
  auto covar_vector = params->GetListListFloat("Covariance", "Covariance of multivariate distribution", {{1.0, 0.5, 0.5}, {0.2, 1.0, 0.1}, {0.1, 0.1, 1.0}});
  for (const auto& row: covar_vector) {
    BARK_EXPECT_TRUE(covar_vector.size() == row.size());
  }

  Eigen::MatrixXd covar(covar_vector.size(), covar_vector.size());
  for (int i = 0; i < covar_vector.size();  ++i) {
    covar.row(i) = Eigen::VectorXd::Map(&covar_vector[i][0],covar_vector[i].size());
  }
  return covar;
}

inline Eigen::VectorXd MultivariateDistribution::MeanFromParams(const ParamsPtr& params) const {
  auto mean_vector = params->GetListFloat("Mean", "Mean of multivariate distribution" ,{1.0, 2.0, 3.0})
  Eigen::VectorXd mean(mean_vector.data());
  return mean;
}

}  // namespace commons
}  // namespace modules

#endif  // MODULES_COMMONS_DISTRIBUTION_MULTIVARIATE_NORMAL_HPP_
