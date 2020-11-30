// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <memory>
#include <optional>
#include <tuple>

#include "bark/models/behavior/behavior_rss/behavior_rss.hpp"
#include "bark/world/observed_world.hpp"
#include "bark/models/dynamic/single_track.hpp"

namespace bark {
namespace models {
namespace behavior {

Trajectory BehaviorRSSConformant::Plan(
    double min_planning_time, const world::ObservedWorld& observed_world) {
  SetBehaviorStatus(BehaviorStatus::VALID);

  const auto& lane_corr = observed_world.GetLaneCorridor();
  if (!initial_lane_corr_) {
    initial_lane_corr_ = lane_corr;
    behavior_safety_model_->SetInitialLaneCorridor(lane_corr);
    auto road_corr = observed_world.GetRoadCorridor();
    const auto& lane_corrs = road_corr->GetUniqueLaneCorridors();
    VLOG(4) << "Initial LaneCorridor used for safety behavior: " << *lane_corr << std::endl;

    // set the other lane corridor as target
    for (const auto& lc : lane_corrs) {
      VLOG(4) << "LaneCorridor: " << *lc << std::endl;
      if (lane_corr != lc) {
        VLOG(4) << "Setting LaneCorridor for nominal behavior: " << *lc << std::endl;
        auto nominal_behavior =
            std::dynamic_pointer_cast<BehaviorIDMLaneTracking>(
                nominal_behavior_model_);
        nominal_behavior->SetConstantLaneCorridor(lc);
      }
    }
  }

  const float length_until_end =
      behavior_safety_model_->GetInitialLaneCorridor()->LengthUntilEnd(
          observed_world.CurrentEgoPosition());
  if (length_until_end <= minimum_safety_corridor_length_) {
    // Do not switch the lane corridor any more but only apply braking in the
    // current lane corridor
    behavior_safety_model_->SetInitialLaneCorridor(lane_corr);
  }

  if (!lane_corr) {
    VLOG(4) << "Agent " << observed_world.GetEgoAgentId()
            << ": Behavior status has expired!" << std::endl;
    SetBehaviorStatus(BehaviorStatus::EXPIRED);
    return GetLastTrajectory();
  }

  auto eval_res =
      boost::get<std::optional<bool>>(rss_evaluator_->Evaluate(observed_world));

#ifdef RSS
  auto rss_evaluator = std::dynamic_pointer_cast<EvaluatorRSS>(rss_evaluator_);
  if (rss_evaluator) {
    const auto& rss_response = rss_evaluator->GetRSSProperResponse();
    lon_response_ = rss_response.longitudinalResponse;
    lat_left_response_ = rss_response.lateralResponseLeft;
    lat_right_response_ = rss_response.lateralResponseRight;
    acc_restrictions_ = rss_response.accelerationRestrictions;
    safety_polygons_ = rss_evaluator->GetSafetyPolygons();
  }
#endif

  if (!*eval_res) {
    VLOG(4) << "RSS is violated." << std::endl;
    behavior_rss_status_ = BehaviorRSSConformantStatus::SAFETY_BEHAVIOR;
    world_time_of_last_rss_violation_ = observed_world.GetWorldTime();
  } else {
    behavior_rss_status_ = BehaviorRSSConformantStatus::NOMINAL_BEHAVIOR;
  }

  Action last_action;
  dynamic::Trajectory last_traj;
  if (behavior_rss_status_ == BehaviorRSSConformantStatus::NOMINAL_BEHAVIOR) {
    // execute normal
    #ifdef RSS
    ApplyRestrictionsToNominalModel(acc_restrictions_);
    #endif
    nominal_behavior_model_->Plan(min_planning_time, observed_world);
    last_action = nominal_behavior_model_->GetLastAction();
    last_traj = nominal_behavior_model_->GetLastTrajectory();
  } else {
    LOG(INFO) << "Executing safety behavior." << std::endl;
    behavior_safety_model_->Plan(min_planning_time, observed_world);
    last_action = behavior_safety_model_->GetLastAction();
    last_traj = behavior_safety_model_->GetLastTrajectory();
  }
  SetLastTrajectory(last_traj);
  SetLastAction(last_action);
  return last_traj;
}

#ifdef RSS
void BehaviorRSSConformant::ApplyRestrictionsToNominalModel(const ::ad::rss::state::AccelerationRestriction& acc_restrictions) {
  bark::models::dynamic::AccelerationLimits acc_lim;
  acc_lim.lat_acc_left_max = acc_restrictions_.lateralLeftRange.maximum;
  acc_lim.lat_acc_right_max = acc_restrictions_.lateralRightRange.maximum;
  acc_lim.lon_acc_max = acc_restrictions_.longitudinalRange.maximum;
  acc_lim.lon_acc_min = acc_restrictions_.longitudinalRange.minimum;
  // TODO: Do we need minimum values as well?
  VLOG(4) << "RSS Response Acceleration Restrictions " << acc_restrictions_;
  VLOG(4) << "AccelerationLimits for IDM " << acc_lim;
  auto nominal_behavior =
          std::dynamic_pointer_cast<BehaviorIDMLaneTracking>(
            nominal_behavior_model_);
  nominal_behavior->SetAccelerationLimits(acc_lim);
}
#endif

}  // namespace behavior
}  // namespace models
}  // namespace bark
