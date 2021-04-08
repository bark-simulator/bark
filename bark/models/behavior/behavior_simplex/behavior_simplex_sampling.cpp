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

Trajectory BehaviorSimplexSampling::Plan(
    double min_planning_time, const world::ObservedWorld& observed_world) {
  SetBehaviorStatus(BehaviorStatus::VALID);

  unsigned num_violations = 0;
  // 1. Sampling of observed worlds from observer (modeling ground truth distribution)
  const auto observed_world_ptr = std::make_shared<ObservedWorld>(observed_world);
    
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
  } else {
    nominal_behavior_model_->Plan(min_planning_time, observed_world);
    last_action = nominal_behavior_model_->GetLastAction();
    last_traj = nominal_behavior_model_->GetLastTrajectory();
  }
  SetLastTrajectory(last_traj);
  SetLastAction(last_action);
  return last_traj;
}


}  // namespace behavior
}  // namespace models
}  // namespace bark
