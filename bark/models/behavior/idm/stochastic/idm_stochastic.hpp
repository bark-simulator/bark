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

typedef enum YieldIntent {
  NO_YIELD = 0,
  YIELD = 1,
  NOT_INITIALIZED = 2
} YieldIntent;

typedef std::unordered_map<
    std::string, std::pair<commons::RandomVariableValueType, commons::RandomVariableValueType>> ParameterRegions;
class BehaviorIDMStochastic : public BehaviorIDMClassic {
 public:
  explicit BehaviorIDMStochastic(const commons::ParamsPtr& params);

  virtual ~BehaviorIDMStochastic() {}

  virtual Trajectory Plan(double delta_time,
                          const ObservedWorld& observed_world);

  virtual std::shared_ptr<BehaviorModel> Clone() const;

  void SampleParameters();

  void ChangeSeed(const bark::commons::RandomSeed& new_seed);

  ParameterRegions GetParameterRegions() const;

  void HandleIntentionChange(const double& world_time);

 protected:
  // represents distributions over standard IDM parameters
  bark::commons::DistributionPtr param_dist_headway_;
  bark::commons::DistributionPtr param_dist_spacing_;
  bark::commons::DistributionPtr param_dist_max_acc_;
  bark::commons::DistributionPtr param_dist_desired_vel_;
  bark::commons::DistributionPtr param_dist_comft_braking_;
  bark::commons::DistributionPtr param_dist_coolness_factor_;

  // Handles stochastic change of yielding intent
  bool use_intention_mechanism_;
  YieldIntent current_yield_intent_;
  double duration_until_intent_change_;
  double world_time_at_last_intent_change_;
  bark::commons::DistributionPtr param_yielding_duration_;
  bark::commons::DistributionPtr param_no_yielding_duration_;

  const double k_max_lat_diff_no_yield = 1.0;
  const double k_max_lat_diff_yield = 8.0;
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
