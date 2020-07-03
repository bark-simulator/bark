// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/models/behavior/motion_primitives/continuous_actions.hpp"
#include "bark/models/dynamic/integration.hpp"
#include "bark/world/observed_world.hpp"

namespace bark {
namespace models {
namespace behavior {

using bark::models::dynamic::StateDefinition;

Trajectory BehaviorMPContinuousActions::Plan(
    float delta_time, const world::ObservedWorld& observed_world) {
  SetBehaviorStatus(BehaviorStatus::VALID);
  const float dt = integration_time_delta_;
  const int num_trajectory_points =
      static_cast<int>(std::ceil(delta_time / dt)) + 1;

  Trajectory traj(num_trajectory_points,
                  static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  traj.row(0) = observed_world.CurrentEgoState();

  float integration_time;
  for (int i = 1; i < num_trajectory_points; ++i) {
    if (i == num_trajectory_points - 1) {
      // calculate the last time pt, which might not fit to dt
      integration_time = delta_time - (i - 1) * dt;
    } else {
      integration_time = dt;
    }
    Input input = GetAction();
    traj.row(i) =
        dynamic::euler_int(*observed_world.GetEgoAgent()->GetDynamicModel(),
                           traj.row(i - 1), input, integration_time);
  }

  SetLastAction(Action(active_motion_));
  this->SetLastTrajectory(traj);
  return traj;
}

BehaviorMotionPrimitives::MotionIdx
BehaviorMPContinuousActions::AddMotionPrimitive(const Input& dynamic_input) {
  motion_primitives_.push_back(dynamic_input);
  return motion_primitives_.size() - 1;
}

}  // namespace behavior
}  // namespace models
}  // namespace bark
