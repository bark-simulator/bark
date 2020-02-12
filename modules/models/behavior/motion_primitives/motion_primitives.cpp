// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/models/behavior/motion_primitives/motion_primitives.hpp"
#include "modules/models/dynamic/integration.hpp"
#include "modules/world/observed_world.hpp"

namespace modules {
namespace models {
namespace behavior {

using modules::models::dynamic::StateDefinition;

BehaviorMotionPrimitives::BehaviorMotionPrimitives(
    const DynamicModelPtr& dynamic_model, commons::Params* params)
    : BehaviorModel(params),
      dynamic_model_(dynamic_model),
      motion_primitives_(),
      active_motion_(),
      integration_time_delta_(params->GetReal(
          "integration_time_delta",
          "the size of the time steps used within the euler integration loop",
          0.02)) {}

Trajectory BehaviorMotionPrimitives::Plan(
    float delta_time, const world::ObservedWorld& observed_world) {
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
    traj.row(i) = dynamic::euler_int(*dynamic_model_, traj.row(i - 1),
                                     motion_primitives_[active_motion_],
                                     integration_time);
  }

  SetLastAction(Action(DiscreteAction(active_motion_)));

  this->SetLastTrajectory(traj);
  return traj;
}

BehaviorMotionPrimitives::MotionIdx
BehaviorMotionPrimitives::AddMotionPrimitive(const Input& dynamic_input) {
  motion_primitives_.push_back(dynamic_input);
  return motion_primitives_.size() - 1;
}

void BehaviorMotionPrimitives::ActionToBehavior(const MotionIdx& motion_idx) {
  active_motion_ = motion_idx;
}

}  // namespace behavior
}  // namespace models
}  // namespace modules
