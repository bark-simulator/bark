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
            "FixedValue", "Value always returned when called sample", {1.0})) {
  }

  virtual RandomVariate Sample() { return fixed_value_; }

  virtual Probability Density(const RandomVariate& variate) const {
    return 0.0;
  };

  virtual Probability CDF(const RandomVariate& variate) const { return 0.0; };

  virtual RandomVariableSupport GetSupport() const {
    return RandomVariableSupport(
        1, std::make_pair(fixed_value_[0], fixed_value_[0]));
  };

  virtual void ChangeSeed(const RandomSeed& new_seed) {}

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
  virtual Probability Density(const RandomVariate& variate) const;
  virtual Probability CDF(const RandomVariate& variate) const;
  virtual RandomVariableSupport GetSupport() const;

  BoostDistType DistFromParams(const ParamsPtr& params) const;

  virtual void ChangeSeed(const RandomSeed& new_seed) {
    seed_ = new_seed;
    generator_.seed(seed_);
  }

 private:
  RandomSeed seed_;
  std::mt19937 generator_;
  BoostDistType dist_;
  std::uniform_real_distribution<RandomVariableValueType> uniform_generator_;
};

using boost_normal = boost::math::normal_distribution<RandomVariableValueType>;
using boost_uniform =
    boost::math::uniform_distribution<RandomVariableValueType>;
using boost_bernoulli = boost::math::bernoulli_distribution<RandomVariableValueType>;
using boost_discrete = std::discrete_distribution<int>;

template <>
inline RandomVariate BoostDistribution1D<boost_uniform>::Sample() {
  // Boost does not provide direct sampling, but we can go over the quantile
  // function
  auto probability = uniform_generator_(generator_);
  auto sample = boost::math::quantile(dist_, probability);
  return RandomVariate(1, sample);
}

template <>
inline RandomVariate BoostDistribution1D<boost_normal>::Sample() {
  // Boost does not provide direct sampling, but we can go over the quantile
  // function
  auto probability = uniform_generator_(generator_);
  auto sample = boost::math::quantile(dist_, probability);
  return RandomVariate(1, sample);
}

template <>
inline RandomVariate BoostDistribution1D<boost_bernoulli>::Sample() {
  // Boost does not provide direct sampling, but we can go over the quantile
  // function
  auto probability = uniform_generator_(generator_);
  auto sample = boost::math::quantile(dist_, probability);
  return RandomVariate(1, sample);
}

template <>
inline RandomVariate BoostDistribution1D<boost_discrete>::Sample() {
  auto sample = dist_(generator_);
  return RandomVariate(1, sample);
}

template <>
inline Probability BoostDistribution1D<boost_uniform>::Density(const RandomVariate& variate) const {
    return boost::math::pdf(dist_, variate[0]);
};

template <>
inline Probability BoostDistribution1D<boost_normal>::Density(const RandomVariate& variate) const {
    return boost::math::pdf(dist_, variate[0]);
};

template <>
inline Probability BoostDistribution1D<boost_bernoulli>::Density(const RandomVariate& variate) const {
    return boost::math::pdf(dist_, variate[0]);
};

template <>
inline double BoostDistribution1D<boost_discrete>::Density(const RandomVariate& variate) const {
    return dist_.probabilities()[variate[0]];
};

template <>
inline Probability BoostDistribution1D<boost_uniform>::CDF(const RandomVariate& variate) const {
    return boost::math::cdf(dist_, variate[0]);
};

template <>
inline Probability BoostDistribution1D<boost_normal>::CDF(const RandomVariate& variate) const {
    return boost::math::cdf(dist_, variate[0]);
};

template <>
inline Probability BoostDistribution1D<boost_bernoulli>::CDF(const RandomVariate& variate) const {
    return boost::math::cdf(dist_, variate[0]);
};

template <>
inline double BoostDistribution1D<boost_discrete>::CDF(const RandomVariate& variate) const {
    double cdf = 0;
    for (int j = 0; j < variate[0]; ++j) {
      cdf += dist_.probabilities()[j];
    }
    return cdf;
};

template <>
inline boost_uniform BoostDistribution1D<boost_uniform>::DistFromParams(
    const ParamsPtr& params) const {
  const RandomVariableValueType lower_bound =
      params->GetReal("LowerBound", "Lower bound of uniform distr.", 0.0);
  const RandomVariableValueType upper_bound =
      params->GetReal("UpperBound", "Upper bound of uniform distr.", 1.0);
  return boost_uniform(lower_bound, upper_bound);
}

template <>
inline boost_normal BoostDistribution1D<boost_normal>::DistFromParams(
    const ParamsPtr& params) const {
  const RandomVariableValueType mean =
      params->GetReal("Mean", "Mean of normal distribution", 0.0);
  const RandomVariableValueType std = params->GetReal(
      "StdDev", "Standard deviation of normal distribution", 1.0);
  return boost_normal(mean, std);
}

template <>
inline boost_bernoulli BoostDistribution1D<boost_bernoulli>::DistFromParams(
    const ParamsPtr& params) const {
  const RandomVariableValueType probability =
      params->GetReal("Probability", "Probability in Bernoulli distribution", 0.5);
  return boost_bernoulli(probability);
}

template <>
inline boost_discrete BoostDistribution1D<boost_discrete>::DistFromParams(
    const ParamsPtr& params) const {
  const std::vector<int> weights =
      params->GetListInt("Weights", "Weight of occurence of Discrete Values", {2, 2, 1});
  return boost_discrete(weights.begin(), weights.end());
}


template <>
inline RandomVariableSupport BoostDistribution1D<boost_uniform>::GetSupport()
    const {
  return RandomVariableSupport(1, std::make_pair(dist_.lower(), dist_.upper()));
}

template <>
inline RandomVariableSupport BoostDistribution1D<boost_normal>::GetSupport()
    const {
  return RandomVariableSupport(
      1, std::make_pair(std::numeric_limits<RandomVariableValueType>::lowest(),
                        std::numeric_limits<RandomVariableValueType>::max()));
}

template <>
inline RandomVariableSupport BoostDistribution1D<boost_bernoulli>::GetSupport()
    const {
  return RandomVariableSupport(
      1, std::make_pair(0,1));
}

template <>
inline RandomVariableSupport BoostDistribution1D<boost_discrete>::GetSupport() const {
    std::pair<int, int> support_1d{
        0,
        std::numeric_limits<int>::max()};
    return RandomVariableSupport(dist_.probabilities().size(), support_1d);
  }


using NormalDistribution1D = BoostDistribution1D<boost_normal>;
using UniformDistribution1D = BoostDistribution1D<boost_uniform>;
using BernoulliDistribution1D = BoostDistribution1D<boost_bernoulli>;
using DiscreteDistribution1D = BoostDistribution1D<boost_discrete>;

}  // namespace commons
}  // namespace bark

#endif  // BARK_COMMONS_DISTRIBUTION_DISTRIBUTIONS_1D_HPP_
