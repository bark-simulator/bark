// Copyright (c) 2020 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/commons/params/params.hpp"
#include "modules/commons/distribution/distributions_1d.hpp"
#include <string>

namespace modules {
namespace commons {

#define GET_DISTRIBUTION_IF_TYPE(type, distribution, params_ptr) \
    if (distribution.compare(#type) == 0) {         \
      return std::make_shared<type>(params_ptr);         \
    }                                                \


DistributionPtr Params::GetDistributionFromType(const std::string& distribution_type, const ParamsPtr& distr_params) const {
  GET_DISTRIBUTION_IF_TYPE(UniformDistribution1D, distribution_type, distr_params)
  GET_DISTRIBUTION_IF_TYPE(NormalDistribution1D, distribution_type, distr_params)
  LOG(ERROR) << "Unknown distribution type";
  throw;
}


}  // namespace commons
}  // namespace modules