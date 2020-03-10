// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_COMMONS_DISTRIBUTION_HPP_
#define MODULES_COMMONS_DISTRIBUTION_HPP_


#include <Eigen/Core>

namespace modules {
namespace commons {

typedef double Probability;
typedef float RandomVariableSupport;
typedef Eigen::Matrix<RandomVariableSupport, Eigen::Dynamic, 1> RandomSample;

class Distribution : public BaseType{
  public:
    Distribution(const ParamsPtr& params) : BaseType(params) {}

    virtual RandomSample sample() = 0;

    virtual Probability GetProbability(const RandomSample& random_sample) const = 0;
};



}  // namespace commons
}  // namespace modules

#endif  // MODULES_COMMONS_DISTRIBUTION_HPP_
