// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
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

BehaviorMotionPrimitives::BehaviorMotionPrimitives(const DynamicModelPtr& dynamic_model, commons::Params *params) : 
    BehaviorModel(params),
    dynamic_model_(dynamic_model),
    motion_primitives_(),
    active_motion_(),
    integration_time_delta_(params->GetReal("integration_time_delta",
                                             "the size of the time steps used within the euler integration loop", 0.02))
    {}

dynamic::Trajectory BehaviorMotionPrimitives::Plan(
    float delta_time, const world::ObservedWorld& observed_world) {
  dynamic::State ego_vehicle_state = observed_world.CurrentEgoState();
  double start_time = observed_world.GetWorldTime();
  const float dt = integration_time_delta_;
  const int num_trajectory_points =
      static_cast<int>(std::ceil(delta_time / dt)) + 1;

  dynamic::Trajectory traj(num_trajectory_points, int(dynamic::StateDefinition::MIN_STATE_SIZE));

  
  int traj_idx=0;
  // set first state as start of trajectory
  traj(traj_idx, StateDefinition::TIME_POSITION) = start_time;
  traj(traj_idx, StateDefinition::X_POSITION) = ego_vehicle_state(StateDefinition::X_POSITION);
  traj(traj_idx, StateDefinition::Y_POSITION) = ego_vehicle_state(StateDefinition::Y_POSITION);
  traj(traj_idx, StateDefinition::THETA_POSITION) = ego_vehicle_state(StateDefinition::THETA_POSITION);
  traj(traj_idx, StateDefinition::VEL_POSITION) = ego_vehicle_state(StateDefinition::VEL_POSITION); 
  
  auto old_state = ego_vehicle_state; // only for getting type, todo: improve

  float integration_time;
  for (++traj_idx; traj_idx<num_trajectory_points; ++traj_idx) {
    old_state(StateDefinition::TIME_POSITION) = traj(traj_idx-1, StateDefinition::TIME_POSITION);
    old_state(StateDefinition::X_POSITION) = traj(traj_idx-1, StateDefinition::X_POSITION);
    old_state(StateDefinition::Y_POSITION) = traj(traj_idx-1, StateDefinition::Y_POSITION);
    old_state(StateDefinition::THETA_POSITION) = traj(traj_idx-1, StateDefinition::THETA_POSITION);
    old_state(StateDefinition::VEL_POSITION) = traj(traj_idx-1, StateDefinition::VEL_POSITION);

    if (traj_idx == num_trajectory_points - 1) {
      // calculate the last time pt, which might not fit to dt
      integration_time = delta_time - (traj_idx - 1) * dt;
    }
    else {
      integration_time = dt;
    }

    auto state = dynamic::euler_int(*dynamic_model_, old_state, motion_primitives_[active_motion_], integration_time);
    traj(traj_idx, StateDefinition::TIME_POSITION) = start_time + (traj_idx - 1) * dt + integration_time;
    traj(traj_idx, StateDefinition::X_POSITION) = state(StateDefinition::X_POSITION);
    traj(traj_idx, StateDefinition::Y_POSITION) = state(StateDefinition::Y_POSITION);
    traj(traj_idx, StateDefinition::THETA_POSITION) = state(StateDefinition::THETA_POSITION);
    traj(traj_idx, StateDefinition::VEL_POSITION) = state(StateDefinition::VEL_POSITION);
  }

  SetLastAction(Action(DiscreteAction(active_motion_)));

  VLOG(2) << "Motion primitive " << active_motion_ << " with input " << motion_primitives_[active_motion_] << " gives 
       trajectory " << traj;

  this->SetLastTrajectory(traj);
  return traj;
}

BehaviorMotionPrimitives::MotionIdx BehaviorMotionPrimitives::AddMotionPrimitive(const Input& dynamic_input) {  
  motion_primitives_.push_back(dynamic_input);
  return motion_primitives_.size()-1;
}

void BehaviorMotionPrimitives::ActionToBehavior(const MotionIdx& motion_idx) {
  active_motion_ = motion_idx;
}

}  // namespace behavior
}  // namespace models
}  // namespace modules
