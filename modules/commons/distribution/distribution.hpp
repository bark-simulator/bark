// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_COMMONS_DISTRIBUTION_HPP_
#define MODULES_COMMONS_DISTRIBUTION_HPP_


#include <Eigen/Core>
#include <vector>

namespace modules {
namespace commons {

typedef double Probability;
typedef float RandomVariableSupport;
typedef unsigned int RandomSeed;
typedef std::vector<RandomVariableSupport> RandomSample;

class Distribution : public BaseType{
  public:
    Distribution(const ParamsPtr& params) : BaseType(params) {}

    virtual RandomSample Sample() = 0;
};

std::shared_ptr<Distribution> DistributionPtr;


}  // namespace commons
}  // namespace modules

#endif  // MODULES_COMMONS_DISTRIBUTION_HPP_
