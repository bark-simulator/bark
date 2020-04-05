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

class BehaviorIDMStochastic : public BehaviorIDMClassic {
 public:
  explicit BehaviorIDMStochastic(const commons::ParamsPtr& params);

  virtual ~BehaviorIDMStochastic() {}

  virtual Trajectory Plan(float delta_time, const ObservedWorld& observed_world);

  virtual std::shared_ptr<BehaviorModel> Clone() const;

  void SampleParameters(); 

  protected:
    modules::commons::DistributionPtr param_dist_headway_;
    modules::commons::DistributionPtr param_dist_spacing_;
    modules::commons::DistributionPtr param_dist_max_acc_;
    modules::commons::DistributionPtr param_dist_desired_vel_;
    modules::commons::DistributionPtr param_dist_comft_braking_;
};

inline std::shared_ptr<BehaviorModel> BehaviorIDMStochastic::Clone() const {
  std::shared_ptr<BehaviorIDMStochastic> model_ptr =
      std::make_shared<BehaviorIDMStochastic>(*this);
  return model_ptr;
}

}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_IDM_STOCHASTIC_idm_stochastic_HPP_
