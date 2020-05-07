// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_IDM_IDM_LANE_TRACKING_HPP_
#define MODULES_MODELS_BEHAVIOR_IDM_IDM_LANE_TRACKING_HPP_

#include <memory>
#include <tuple>
#include "modules/models/behavior/idm/base_idm.hpp"

namespace modules {
namespace models {
namespace behavior {

class BehaviorIDMLaneTracking : public BaseIDM {
 public:
  explicit BehaviorIDMLaneTracking(const commons::ParamsPtr& params)
      : BaseIDM(params) {
    crosstrack_error_gain_ = params->GetReal(
      "BehaviorIDMLaneTracking::CrosstrackErrorGain",
      "Tuning factor of stanley controller",
      1.0);
  }

  virtual ~BehaviorIDMLaneTracking() {}

  std::tuple<Trajectory, Action> GenerateTrajectory(
    const world::ObservedWorld& observed_world,
    const LaneCorridorPtr& lane_corr,
    const std::tuple<double, double, bool>& rel_values,
    float delta_time) const;
  
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
