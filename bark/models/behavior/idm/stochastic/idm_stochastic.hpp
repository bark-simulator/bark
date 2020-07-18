// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_MODELS_BEHAVIOR_IDM_STOCHASTIC_HPP_
#define BARK_MODELS_BEHAVIOR_IDM_STOCHASTIC_HPP_

#include "bark/commons/distribution/distributions_1d.hpp"
#include "bark/models/behavior/idm/idm_classic.hpp"

namespace bark {
namespace models {
namespace behavior {

class BehaviorIDMStochastic : public BehaviorIDMClassic {
 public:
  explicit BehaviorIDMStochastic(const commons::ParamsPtr& params);

  virtual ~BehaviorIDMStochastic() {}

  virtual Trajectory Plan(float delta_time,
                          const ObservedWorld& observed_world);

  virtual std::shared_ptr<BehaviorModel> Clone() const;

  void SampleParameters();

 protected:
  bark::commons::DistributionPtr param_dist_headway_;
  bark::commons::DistributionPtr param_dist_spacing_;
  bark::commons::DistributionPtr param_dist_max_acc_;
  bark::commons::DistributionPtr param_dist_desired_vel_;
  bark::commons::DistributionPtr param_dist_comft_braking_;
  bark::commons::DistributionPtr param_dist_coolness_factor_;
};

inline std::shared_ptr<BehaviorModel> BehaviorIDMStochastic::Clone() const {
  std::shared_ptr<BehaviorIDMStochastic> model_ptr =
      std::make_shared<BehaviorIDMStochastic>(*this);
  return model_ptr;
}

}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // BARK_MODELS_BEHAVIOR_IDM_STOCHASTIC_HPP_
