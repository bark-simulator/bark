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
#include "bark/models/behavior/behavior_simplex/behavior_simplex_sampling.hpp"
#include "bark/models/dynamic/single_track.hpp"
#include "bark/world/observed_world.hpp"

namespace bark {
namespace models {
namespace behavior {

bool BehaviorSimplexSampling::PreprocessLaneInformation(const world::ObservedWorld& observed_world) {
  const auto& lane_corr = observed_world.GetLaneCorridor();
  if (!initial_lane_corr_) {
    initial_lane_corr_ = lane_corr;
    behavior_safety_model_->SetInitialLaneCorridor(lane_corr);
    auto road_corr = observed_world.GetRoadCorridor();
    const auto& lane_corrs = road_corr->GetUniqueLaneCorridors();
    VLOG(4) << "Initial LaneCorridor used for safety behavior: " << *lane_corr
            << std::endl;

    // set the other lane corridor as target
    for (const auto& lc : lane_corrs) {
      VLOG(4) << "LaneCorridor: " << *lc << std::endl;
      if (lane_corr != lc) {
        VLOG(4) << "Setting LaneCorridor for nominal behavior: " << *lc
                << std::endl;
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

  if (!lane_corr) return false; else return true;
}

BehaviorRSSConformantStatus BehaviorSimplexSampling::GetBehaviorRssStatus() const {
  return behavior_rss_status_;
}

Trajectory BehaviorSimplexSampling::Plan(
    double min_planning_time, const world::ObservedWorld& observed_world) {
  SetBehaviorStatus(BehaviorStatus::VALID);

  unsigned num_violations = 0;
  // 1. Sampling of observed worlds from observer (modeling ground truth distribution)
  const auto observed_world_ptr = std::make_shared<ObservedWorld>(observed_world);

  bool preprocess_lane_res = PreprocessLaneInformation(observed_world);

  if (!preprocess_lane_res) {
    VLOG(4) << "Agent " << observed_world.GetEgoAgentId()
            << ": Behavior status has expired!" << std::endl;
    SetBehaviorStatus(BehaviorStatus::EXPIRED);
    return GetLastTrajectory();
  }

#ifdef RSS
  for( unsigned int sample_idx = 0; sample_idx < num_samples_; ++sample_idx ) {
    const auto sampled_observed_world = observed_world.Observe({observed_world.GetEgoAgentId()}).at(0);
    auto eval_res = boost::get<std::optional<bool>>(rss_evaluator_->Evaluate(sampled_observed_world));
    if(*eval_res) {
      num_violations++;
    }
  }
#endif
  current_expected_safety_violation_ = double(num_violations) / double(num_samples_);
  // Violation threshold describes percentage if safety violations within "response time"
  Action last_action;
  dynamic::Trajectory last_traj;
  if (current_expected_safety_violation_ > violation_threshold_) {
    behavior_safety_model_->Plan(min_planning_time, observed_world);
    last_action = behavior_safety_model_->GetLastAction();
    last_traj = behavior_safety_model_->GetLastTrajectory();
    behavior_rss_status_ = BehaviorRSSConformantStatus::SAFETY_BEHAVIOR;
  } else {
    nominal_behavior_model_->Plan(min_planning_time, observed_world);
    last_action = nominal_behavior_model_->GetLastAction();
    last_traj = nominal_behavior_model_->GetLastTrajectory();
    behavior_rss_status_ = BehaviorRSSConformantStatus::NOMINAL_BEHAVIOR;
  }
  SetLastTrajectory(last_traj);
  SetLastAction(last_action);
  return last_traj;
}


}  // namespace behavior
}  // namespace models
}  // namespace bark
