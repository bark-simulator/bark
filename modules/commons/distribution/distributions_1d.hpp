// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_COMMONS_DISTRIBUTION_DISTRIBUTIONS_1D_HPP_
#define MODULES_COMMONS_DISTRIBUTION_DISTRIBUTIONS_1D_HPP_

#include <random>
#include <boost/random.hpp>
#include <boost/math/distributions.hpp>

#include "modules/commons/distribution/distribution.hpp"
#include "modules/commons/params/params.hpp"


namespace modules {
namespace commons {


typedef double RandomVariate1D;
std::shared_ptr<Distribution<RandomVariate1D>> Distribution1DPtr;

template<class BoostDistType>
class BoostDistribution1D : public Distribution<RandomVariate1D>  {
  public:
    BoostDistribution1D(const ParamsPtr& params) : Distribution<RandomVariate1D>(params),
             seed_(params->GetInt("RandomSeed", "Specifies seed for mersenne twister engine", 1234)),
             generator_(seed_),
             dist_(DistFromParams(params)),
             uniform_generator_(0, 1.0) {}

    virtual RandomVariate1D Sample();

    virtual Probability Density(const RandomVariate1D& variate) const {
          return boost::math::pdf(dist_, variate);
          };

    virtual Probability CDF(const RandomVariate1D& variate) const {
      return boost::math::cdf(dist_, variate); 
      }; 

    BoostDistType DistFromParams(const ParamsPtr& params) const;

  private:
    RandomSeed seed_;
    std::mt19937 generator_;
    BoostDistType dist_;
    std::uniform_real_distribution<RandomVariableSupport> uniform_generator_;
};


template<class BoostDistType>
RandomVariate1D BoostDistribution1D<BoostDistType>::Sample() {
  // Boost does not provide direct sampling, but we can go over the quantile function
  auto probability = uniform_generator_(generator_);
  return boost::math::quantile(dist_, probability);
}

using boost_normal = boost::math::normal_distribution<RandomVariableSupport>;
using boost_uniform = boost::math::uniform_distribution<RandomVariableSupport>;

template<>
boost_uniform BoostDistribution1D<boost_uniform>::DistFromParams(const ParamsPtr& params) const {
  const RandomVariableSupport lower_bound = params->GetReal("LowerBound", "Lower bound of uniform distr.", 0.0f);
  const RandomVariableSupport upper_bound = params->GetReal("UpperBound", "Lower bound of uniform distr.", 1.0f);
  return boost_uniform(lower_bound, upper_bound);
}

template<>
boost_normal BoostDistribution1D<boost_normal>::DistFromParams(const ParamsPtr& params) const {
  const RandomVariableSupport mean = params->GetReal("Mean", "Mean of normal distribution", 0.0f);
  const RandomVariableSupport std = params->GetReal("StdDev", "Standard deviation of normal distribution", 1.0f);
  return boost_normal(mean, std);
}

using NormalDistribution1D = BoostDistribution1D<boost_normal>;
using UniformDistribution1D = BoostDistribution1D<boost_normal>;


}  // namespace commons
}  // namespace modules

#endif  // MODULES_COMMONS_DISTRIBUTION_DISTRIBUTIONS_1D_HPP_
