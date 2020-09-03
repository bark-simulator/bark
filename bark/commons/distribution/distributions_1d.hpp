// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_COMMONS_DISTRIBUTION_DISTRIBUTIONS_1D_HPP_
#define BARK_COMMONS_DISTRIBUTION_DISTRIBUTIONS_1D_HPP_

#include <boost/math/distributions.hpp>
#include <boost/random.hpp>
#include <random>

#include "bark/commons/distribution/distribution.hpp"
#include "bark/commons/params/params.hpp"

namespace bark {
namespace commons {

class FixedValue : public Distribution {
 public:
  FixedValue(const ParamsPtr& params)
      : Distribution(params),
        fixed_value_(params->GetListFloat(
            "FixedValue", "Value always returned when called sample", {1.0f})) {
  }

  virtual RandomVariate Sample() { return fixed_value_; }

  virtual Probability Density(const RandomVariate& variate) const {
    return 0.0f;
  };

  virtual Probability CDF(const RandomVariate& variate) const { return 0.0f; };

  virtual RandomVariableSupport GetSupport() const { return RandomVariableSupport(1,
                                                  std::make_pair(fixed_value_[0], fixed_value_[0])); };

 private:
  RandomVariate fixed_value_;
};

template <class BoostDistType>
class BoostDistribution1D : public Distribution {
 public:
  BoostDistribution1D(const ParamsPtr& params)
      : Distribution(params),
        seed_(params->GetInt(
            "RandomSeed", "Specifies seed for mersenne twister engine", 1234)),
        generator_(seed_),
        dist_(DistFromParams(params)),
        uniform_generator_(0, 1.0) {}

  virtual RandomVariate Sample();

  virtual Probability Density(const RandomVariate& variate) const {
    return boost::math::pdf(dist_, variate[0]);
  };

  virtual Probability CDF(const RandomVariate& variate) const {
    return boost::math::cdf(dist_, variate[0]);
  };

  virtual RandomVariableSupport GetSupport() const;

  BoostDistType DistFromParams(const ParamsPtr& params) const;

 private:
  RandomSeed seed_;
  std::mt19937 generator_;
  BoostDistType dist_;
  std::uniform_real_distribution<RandomVariableValueType> uniform_generator_;
};

template <class BoostDistType>
inline RandomVariate BoostDistribution1D<BoostDistType>::Sample() {
  // Boost does not provide direct sampling, but we can go over the quantile
  // function
  auto probability = uniform_generator_(generator_);
  auto sample = boost::math::quantile(dist_, probability);
  return RandomVariate(1, sample);
}

using boost_normal = boost::math::normal_distribution<RandomVariableValueType>;
using boost_uniform = boost::math::uniform_distribution<RandomVariableValueType>;

template <>
inline boost_uniform BoostDistribution1D<boost_uniform>::DistFromParams(
    const ParamsPtr& params) const {
  const RandomVariableValueType lower_bound =
      params->GetReal("LowerBound", "Lower bound of uniform distr.", 0.0f);
  const RandomVariableValueType upper_bound =
      params->GetReal("UpperBound", "Upper bound of uniform distr.", 1.0f);
  return boost_uniform(lower_bound, upper_bound);
}

template <>
inline boost_normal BoostDistribution1D<boost_normal>::DistFromParams(
    const ParamsPtr& params) const {
  const RandomVariableValueType mean =
      params->GetReal("Mean", "Mean of normal distribution", 0.0f);
  const RandomVariableValueType std = params->GetReal(
      "StdDev", "Standard deviation of normal distribution", 1.0f);
  return boost_normal(mean, std);
}

template <>
inline RandomVariableSupport BoostDistribution1D<boost_uniform>::GetSupport() const {
  return RandomVariableSupport(1, std::make_pair(dist_.lower(), dist_.upper()));
}

template <>
inline RandomVariableSupport BoostDistribution1D<boost_normal>::GetSupport() const {
  return RandomVariableSupport(1, std::make_pair
      (std::numeric_limits<RandomVariableValueType>::lowest(), std::numeric_limits<RandomVariableValueType>::max()));
}

using NormalDistribution1D = BoostDistribution1D<boost_normal>;
using UniformDistribution1D = BoostDistribution1D<boost_uniform>;

}  // namespace commons
}  // namespace bark

#endif  // BARK_COMMONS_DISTRIBUTION_DISTRIBUTIONS_1D_HPP_
