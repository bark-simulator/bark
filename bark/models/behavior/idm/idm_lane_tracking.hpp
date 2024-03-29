// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_MODELS_BEHAVIOR_IDM_IDM_LANE_TRACKING_HPP_
#define BARK_MODELS_BEHAVIOR_IDM_IDM_LANE_TRACKING_HPP_

#include <memory>
#include <tuple>
#include "bark/models/behavior/idm/base_idm.hpp"

namespace bark {
namespace models {
namespace behavior {

using bark::world::map::LaneCorridor;
using bark::world::map::LaneCorridorPtr;

// IDM that follows the centerline of a LaneCorridor
// using the dynamic SingleTrack model
class BehaviorIDMLaneTracking : public BaseIDM {
 public:
  explicit BehaviorIDMLaneTracking(const commons::ParamsPtr& params) :
    BehaviorModel(params),
    BaseIDM(params),
    limit_steering_rate_(params->GetBool("BehaviorIDMLaneTracking::LimitSteeringRate",
                        "Bool if steering limited according to dynamics model", true)) {
    crosstrack_error_gain_ =
        params->GetReal("BehaviorIDMLaneTracking::CrosstrackErrorGain",
                        "Tuning factor of stanley controller", 1.0);
    dynamic::Input input(2);
    input << 0.0, 0.0;
    SetLastAction(input);
  }

  virtual ~BehaviorIDMLaneTracking() {}

  std::tuple<Trajectory, Action> GenerateTrajectory(
      const world::ObservedWorld& observed_world,
      const LaneCorridorPtr& lane_corr, const IDMRelativeValues& rel_values,
      double delta_time) const;

  virtual std::shared_ptr<BehaviorModel> Clone() const;
  void SetLimitSteeringRate(bool limit_steering) {
    limit_steering_rate_ = limit_steering;
  }

  void CheckAccelerationLimits(double acc_lon, double acc_lat) const;

  friend class BaseIDM;

  bool operator==(const BehaviorIDMLaneTracking& other) const {
    return *GetParams()->AddChild("BehaviorIDMLaneTracking") ==
          *other.GetParams()->AddChild("BehaviorIDMLaneTracking") &&
         *GetParams()->AddChild("BehaviorIDMClassic") ==
          *other.GetParams()->AddChild("BehaviorIDMClassic");
  } 

 private:
  double crosstrack_error_gain_;
  bool limit_steering_rate_;
};

inline std::shared_ptr<BehaviorModel> BehaviorIDMLaneTracking::Clone() const {
  std::shared_ptr<BehaviorIDMLaneTracking> model_ptr =
      std::make_shared<BehaviorIDMLaneTracking>(*this);
  return model_ptr;
}

}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // BARK_MODELS_BEHAVIOR_IDM_IDM_LANE_TRACKING_HPP_
