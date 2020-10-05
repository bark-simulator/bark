// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_COMMONS_DISTRIBUTION_MULTIVARIATE_NORMAL_HPP_
#define BARK_COMMONS_DISTRIBUTION_MULTIVARIATE_NORMAL_HPP_

#include <Eigen/Eigen>
#include <random>

#include "bark/commons/distribution/distribution.hpp"
#include "bark/commons/params/params.hpp"

namespace bark {
namespace commons {

// Based on
// https://stackoverflow.com/questions/6142576/sample-from-multivariate-normal-gaussian-distribution-in-c
class MultivariateDistribution : public Distribution {
 public:
  MultivariateDistribution(const ParamsPtr& params)
      : Distribution(params),
        seed_(params->GetInt(
            "RandomSeed", "Specifies seed for mersenne twister engine", 1234)),
        generator_(seed_),
        dist_(),
        mean_(MeanFromParams(params)) {
    auto covariance = CovarFromParams(params);
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eigenSolver(covariance);
    transform_ = eigenSolver.eigenvectors() *
                 eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
  }

  virtual RandomVariate Sample();

  virtual Probability Density(const RandomVariate& variate) const {
    return 0.0f;
  };

  virtual Probability CDF(const RandomVariate& variate) const { return 0.0f; };

  virtual RandomVariableSupport GetSupport() const {
    std::pair<RandomVariableValueType, RandomVariableValueType> 
      support_1d{std::numeric_limits<RandomVariableValueType>::lowest(), std::numeric_limits<RandomVariableValueType>::max()};
    return RandomVariableSupport(mean_.size(), support_1d);
  }

  Eigen::MatrixXf CovarFromParams(const ParamsPtr& params) const;
  Eigen::VectorXf MeanFromParams(const ParamsPtr& params) const;

 private:
  RandomSeed seed_;
  std::mt19937 generator_;
  std::normal_distribution<RandomVariableValueType> dist_;
  Eigen::VectorXf mean_;
  Eigen::MatrixXf transform_;
};

inline RandomVariate MultivariateDistribution::Sample() {
  Eigen::VectorXf eigen_sample =
      mean_ + transform_ * Eigen::VectorXf{mean_.size()}.unaryExpr(
                               [&](auto x) { return dist_(generator_); });
  RandomVariate sample(eigen_sample.data(),
                       eigen_sample.data() + eigen_sample.size());
  return sample;
}

inline Eigen::MatrixXf MultivariateDistribution::CovarFromParams(
    const ParamsPtr& params) const {
  auto covar_vector = params->GetListListFloat(
      "Covariance", "Covariance of multivariate distribution",
      {{1.0, 0.5, 0.5}, {0.2, 1.0, 0.1}, {0.1, 0.1, 1.0}});
  for (const auto& row : covar_vector) {
    BARK_EXPECT_TRUE(covar_vector.size() == row.size());
  }

  Eigen::MatrixXf covar(covar_vector.size(), covar_vector.size());
  for (int i = 0; i < covar_vector.size(); ++i) {
    covar.row(i) = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(
        covar_vector[i].data(), covar_vector[i].size());
  }
  return covar;
}

inline Eigen::VectorXf MultivariateDistribution::MeanFromParams(
    const ParamsPtr& params) const {
  auto mean_vector = params->GetListFloat(
      "Mean", "Mean of multivariate distribution", {1.0, 2.0, 3.0});
  Eigen::VectorXf mean = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(
      mean_vector.data(), mean_vector.size());
  return mean;
}

}  // namespace commons
}  // namespace bark

#endif  // BARK_COMMONS_DISTRIBUTION_MULTIVARIATE_NORMAL_HPP_
