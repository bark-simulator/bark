// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_COMMONS_DISTRIBUTION_HPP_
#define MODULES_COMMONS_DISTRIBUTION_HPP_


#include <Eigen/Core>
#include <vector>

#include "modules/commons/distribution/distribution.hpp"
#include "modules/commons/params/params.hpp"
#include <random>

namespace modules {
namespace commons {

typedef std::vector<RandomVariableSupport> RandomSample;

template<class StdDistType>
class StdDistribution1D : public Distribution{
  public:
    StdDistribution1D(const ParamsPtr& params) : Distribution(params),
             seed_(params->GetInt("RandomSeed", "Specifies seed for mersenne twister engine", 1234)),
             generator_(seed_),
             dist_() {}

    virtual RandomSample Sample();

    ParamsPtr BarkParamsFromStdParams();

  private:
    RandomSeed seed_;
    std::mt19937 generator_;
    StdDistribution1D dist_();
};

typedef std::shared_ptr<Distribution> DistributionPtr;

template<class StdDistType>
RandomSample StdDistribution1D<StdDistType>::Sample() {
  return dist_(generator_);
}


}  // namespace commons
}  // namespace modules

#endif  // MODULES_COMMONS_DISTRIBUTION_HPP_
