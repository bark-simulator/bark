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
        dist_(){
    mean_ = MeanFromParams(params);
    covariance_ = CovarFromParams(params);
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(covariance_);
    transform_ = eigenSolver.eigenvectors() * // cholesky decomposition
                 eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
  }

  virtual RandomVariate Sample();

  virtual Probability Density(const RandomVariate& variate) const;

  virtual Probability CDF(const RandomVariate& variate) const { return 0.0; };

  virtual RandomVariableSupport GetSupport() const {
    std::pair<RandomVariableValueType, RandomVariableValueType> support_1d{
        std::numeric_limits<RandomVariableValueType>::lowest(),
        std::numeric_limits<RandomVariableValueType>::max()};
    return RandomVariableSupport(mean_.size(), support_1d);
  }

  Eigen::MatrixXd CovarFromParams(const ParamsPtr& params) const;
  Eigen::VectorXd MeanFromParams(const ParamsPtr& params) const;

 private:
  RandomSeed seed_;
  std::mt19937 generator_;
  std::normal_distribution<RandomVariableValueType> dist_;
  Eigen::VectorXd mean_;
  Eigen::MatrixXd covariance_;
  Eigen::MatrixXd transform_;
};

inline RandomVariate MultivariateDistribution::Sample() {
  Eigen::VectorXd eigen_sample =
      mean_ + transform_ * Eigen::VectorXd{mean_.size()}.unaryExpr(
                               [&](auto x) { return dist_(generator_); });
  RandomVariate sample(eigen_sample.data(),
                       eigen_sample.data() + eigen_sample.size());
  return sample;
}

inline Probability MultivariateDistribution::Density(const RandomVariate& variate) const {
  const Eigen::VectorXd x = Eigen::Map<const Eigen::VectorXd>(variate.data(), variate.size());
  double sqrt2pi = std::sqrt(2 * M_PI);
  double quadform  = (x - mean_).transpose() * covariance_.inverse() * (x - mean_);
  double norm = std::pow(sqrt2pi, - x.size()) * std::pow(covariance_.determinant(), -0.5);
  return norm * exp(-0.5 * quadform);
}

inline Eigen::MatrixXd MultivariateDistribution::CovarFromParams(
    const ParamsPtr& params) const {
  auto covar_vector = params->GetListListFloat(
      "Covariance", "Covariance of multivariate distribution",
      {{1.0, 0.5, 0.5}, {0.2, 1.0, 0.1}, {0.1, 0.1, 1.0}});
  for (const auto& row : covar_vector) {
    BARK_EXPECT_TRUE(covar_vector.size() == row.size());
  }

  Eigen::MatrixXd covar(covar_vector.size(), covar_vector.size());
  for (unsigned int i = 0; i < covar_vector.size(); ++i) {
    covar.row(i) = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        covar_vector[i].data(), covar_vector[i].size());
  }
  return covar;
}

inline Eigen::VectorXd MultivariateDistribution::MeanFromParams(
    const ParamsPtr& params) const {
  auto mean_vector = params->GetListFloat(
      "Mean", "Mean of multivariate distribution", {1.0, 2.0, 3.0});
  Eigen::VectorXd mean = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      mean_vector.data(), mean_vector.size());
  return mean;
}

}  // namespace commons
}  // namespace bark

#endif  // BARK_COMMONS_DISTRIBUTION_MULTIVARIATE_NORMAL_HPP_
