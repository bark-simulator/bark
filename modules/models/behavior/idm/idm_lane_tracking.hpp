// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_IDM_IDM_LANE_TRACKING_HPP_
#define MODULES_MODELS_BEHAVIOR_IDM_IDM_LANE_TRACKING_HPP_

#include <memory>

#include "modules/models/behavior/idm/idm_classic.hpp"

namespace modules {
namespace models {
namespace behavior {

class BehaviorIDMLaneTracking : public BehaviorIDMClassic {
 public:
  explicit BehaviorIDMLaneTracking(const commons::ParamsPtr& params)
      : BehaviorIDMClassic(params) {
    crosstrack_error_gain_ = params->GetReal(
        "BehaviorIDMLaneTracking::CrosstrackErrorGain", "Tuning factor of stanley controller", 1.0);
  }

  virtual ~BehaviorIDMLaneTracking() {}

  Trajectory Plan(float delta_time, const ObservedWorld& observed_world);

  virtual std::shared_ptr<BehaviorModel> Clone() const;

 private:
  double crosstrack_error_gain_;
};

inline std::shared_ptr<BehaviorModel> BehaviorIDMLaneTracking::Clone() const {
  std::shared_ptr<BehaviorIDMLaneTracking> model_ptr =
      std::make_shared<BehaviorIDMLaneTracking>(*this);
  return model_ptr;
}

}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_IDM_IDM_LANE_TRACKING_HPP_
