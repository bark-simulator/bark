// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_COMMONS_DISTRIBUTION_DISTRIBUTION_HPP_
#define BARK_COMMONS_DISTRIBUTION_DISTRIBUTION_HPP_

#include <Eigen/Core>
#include <vector>

#include "bark/commons/base_type.hpp"

namespace bark {
namespace commons {

typedef double Probability;
typedef float RandomVariableSupport;
typedef unsigned int RandomSeed;
typedef std::vector<RandomVariableSupport> RandomVariate;

class Distribution : public BaseType {
 public:
  Distribution(const ParamsPtr& params) : BaseType(params) {}

  virtual RandomVariate Sample() = 0;

  virtual Probability Density(const RandomVariate& variate) const = 0;

  virtual Probability CDF(const RandomVariate& variate) const = 0;
};

typedef std::shared_ptr<Distribution> DistributionPtr;

}  // namespace commons
}  // namespace bark

#endif  // BARK_COMMONS_DISTRIBUTION_DISTRIBUTION_HPP_
