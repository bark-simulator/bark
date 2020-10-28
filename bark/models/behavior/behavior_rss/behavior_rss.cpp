// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <tuple>
#include <optional>

#include "bark/models/behavior/behavior_rss/behavior_rss.hpp"
#include "bark/world/observed_world.hpp"

namespace bark {
namespace models {
namespace behavior {

Trajectory BehaviorRSSConformant::Plan(
    float min_planning_time, const world::ObservedWorld& observed_world) {
  SetBehaviorStatus(BehaviorStatus::VALID);

  const auto& lane_corr = observed_world.GetLaneCorridor();
  if (!initial_lane_corr_) {
    initial_lane_corr_ = lane_corr;
    behavior_safety_model_->SetInitialLaneCorridor(lane_corr);
    auto road_corr = observed_world.GetRoadCorridor();
    const auto& lane_corrs = road_corr->GetUniqueLaneCorridors();
    VLOG(4) << "Initial LaneCorridor: " << &lane_corr << std::endl;

    // set the other lane corridor as target
    for (const auto& lc : lane_corrs) {
      VLOG(4) << "LaneCorridor: " << &lc << std::endl;
      if (lane_corr != lc) {
        VLOG(4) << "Setting LaneCorridor: " << &lc << std::endl;
        auto nominal_behavior =
          std::dynamic_pointer_cast<BehaviorIDMLaneTracking>(
            nominal_behavior_model_);
        nominal_behavior->SetConstantLaneCorridor(lc);
      }
    }
  }

  if (!lane_corr) {
    VLOG(4) << "Agent " << observed_world.GetEgoAgentId()
              << ": Behavior status has expired!" << std::endl;
    SetBehaviorStatus(BehaviorStatus::EXPIRED);
    return GetLastTrajectory();
  }

  auto eval_res = boost::get<std::optional<bool>>(
    rss_evaluator_->Evaluate(observed_world));

  auto rss_evaluator = std::dynamic_pointer_cast<EvaluatorRSS>(
    rss_evaluator_);
  if(rss_evaluator) {
    lon_ = rss_evaluator->GetLongitudinalResponse();
    lat_left_ = rss_evaluator->GetLateralLeftResponse();
    lat_right_ = rss_evaluator->GetLateralRightResponse();
    dangerous_objects_ = rss_evaluator->GetDangerousObjectIdsSafetyResponse();
  }

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
    nominal_behavior_model_->Plan(min_planning_time, observed_world);
    last_action = nominal_behavior_model_->GetLastAction();
    last_traj = nominal_behavior_model_->GetLastTrajectory();
  } else {
    // LOG(INFO) << "Executing safety behavior." << std::endl;
    behavior_safety_model_->Plan(min_planning_time, observed_world);
    last_action = behavior_safety_model_->GetLastAction();
    last_traj = behavior_safety_model_->GetLastTrajectory();
  }
  SetLastTrajectory(last_traj);
  SetLastAction(last_action);
  return last_traj;
}


}  // namespace behavior
}  // namespace models
}  // namespace bark
