// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <tuple>
#include <memory>

#include "bark/models/behavior/behavior_safety/behavior_safety.hpp"
#include "bark/models/behavior/idm/idm_lane_tracking.hpp"
#include "bark/world/observed_world.hpp"

namespace bark {
namespace models {
namespace behavior {

Trajectory BehaviorSafety::Plan(
    float min_planning_time, const world::ObservedWorld& observed_world) {
  SetBehaviorStatus(BehaviorStatus::VALID);

  // for now we support only the BehaviorIDMLaneTracking
  BARK_EXPECT_TRUE(std::dynamic_pointer_cast<BehaviorIDMLaneTracking>(behavior_model_));

  std::shared_ptr<BehaviorIDMLaneTracking> idm_lane_tracking_behavior =
    std::dynamic_pointer_cast<BehaviorIDMLaneTracking>(behavior_model_);
  
  idm_lane_tracking_behavior->SetConstantLaneCorridor(initial_lane_corr_);

  idm_lane_tracking_behavior->Plan(min_planning_time, observed_world);
  auto last_action = idm_lane_tracking_behavior->GetLastAction();
  auto last_traj = idm_lane_tracking_behavior->GetLastTrajectory();
  
  // set values
  SetLastTrajectory(last_traj);
  SetLastAction(last_action);
  return last_traj;

}


}  // namespace behavior
}  // namespace models
}  // namespace bark
