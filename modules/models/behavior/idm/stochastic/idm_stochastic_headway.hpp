// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_BEHAVIOR_STOCHASTIC_HPP_
#define MODULES_MODELS_BEHAVIOR_BEHAVIOR_STOCHASTIC_HPP_

#include "modules/models/behavior/idm/idm_classic.hpp"
#include "modules/commons/distribution/distributions_1d.hpp"

namespace modules {
namespace models {
namespace behavior {

class BehaviorIDMStochasticHeadway : public BehaviorIDMClassic {
 public:
  explicit BehaviorIDMStochasticHeadway(const commons::ParamsPtr& params);

  virtual ~BehaviorIDMStochasticHeadway() {}

  Trajectory Plan(float delta_time, const ObservedWorld& observed_world);

  virtual std::shared_ptr<BehaviorModel> Clone() const;

  void SampleParameters(); 

  protected:
    modules::commons::DistributionPtr param_dist_headway_;
   
};

inline std::shared_ptr<BehaviorModel> BehaviorIDMStochasticHeadway::Clone() const {
  std::shared_ptr<BehaviorIDMStochasticHeadway> model_ptr =
      std::make_shared<BehaviorIDMStochasticHeadway>(*this);
  return model_ptr;
}

}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_IDM_STOCHASTIC_IDM_STOCHASTIC_HEADWAY_HPP_
