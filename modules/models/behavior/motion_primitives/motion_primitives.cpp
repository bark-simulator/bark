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

BehaviorMotionPrimitives::BehaviorMotionPrimitives(const DynamicModelPtr& dynamic_model, commons::Params *params) : 
    BehaviorModel(params),
    dynamic_model_(dynamic_model),
    motion_primitives_(),
    active_motion_(),
    integration_time_delta_(params->get_real("integration_time_delta",
                                             "the size of the time steps used within the euler integration loop", 0.05))
    {}

dynamic::Trajectory BehaviorMotionPrimitives::Plan(
    float delta_time,
    const world::ObservedWorld& observed_world) {

  using namespace dynamic;

  //! TODO(@fortiss): parameters
  const int num_traj_time_points = 2;
  dynamic::Trajectory traj(num_traj_time_points, int(StateDefinition::MIN_STATE_SIZE));
  /*
  dynamic::State ego_vehicle_state = observed_world.get_ego_state();
  
  double start_time = observed_world.get_world_time();

  // add current state
  traj(0, StateDefinition::TIME_POSITION) = start_time;
  traj(0, StateDefinition::X_POSITION) = ego_vehicle_state(StateDefinition::X_POSITION);
  traj(0, StateDefinition::Y_POSITION) = ego_vehicle_state(StateDefinition::Y_POSITION);
  traj(0, StateDefinition::THETA_POSITION) = ego_vehicle_state(StateDefinition::THETA_POSITION);
  traj(0, StateDefinition::VEL_POSITION) = ego_vehicle_state(StateDefinition::VEL_POSITION);

  // add current state
  traj(1, StateDefinition::TIME_POSITION) = start_time + delta_time;
  traj(1, StateDefinition::X_POSITION) = ego_vehicle_state(StateDefinition::X_POSITION) + state_delta_(StateDefinition::X_POSITION);
  traj(1, StateDefinition::Y_POSITION) = ego_vehicle_state(StateDefinition::Y_POSITION) + state_delta_(StateDefinition::Y_POSITION);;
  traj(1, StateDefinition::THETA_POSITION) = ego_vehicle_state(StateDefinition::THETA_POSITION) + state_delta_(StateDefinition::THETA_POSITION);;
  traj(1, StateDefinition::VEL_POSITION) = ego_vehicle_state(StateDefinition::VEL_POSITION) + state_delta_(StateDefinition::VEL_POSITION);;

  this->set_last_trajectory(traj);*/
  return traj;
}

BehaviorMotionPrimitives::MotionIdx BehaviorMotionPrimitives::AddMotionPrimitive(const Input& dynamic_input, const float time_span) {
  using modules::models::dynamic::StateDefinition;
  
  const float dt = integration_time_delta_; 
  const int num_trajectory_points = static_cast<int>(std::ceil(time_span / dt));

  dynamic::Trajectory motion_primitive(num_trajectory_points, int(dynamic::StateDefinition::MIN_STATE_SIZE));
  int trajectory_idx = 0;

  motion_primitive.row(trajectory_idx).setZero();
  for (trajectory_idx=1; trajectory_idx<num_trajectory_points-2; ++trajectory_idx) {
    auto old_state = State(int(dynamic::StateDefinition::MIN_STATE_SIZE)); 
    old_state(StateDefinition::TIME_POSITION) = motion_primitive(trajectory_idx-1, StateDefinition::TIME_POSITION);
    old_state(StateDefinition::X_POSITION) = motion_primitive(trajectory_idx-1, StateDefinition::X_POSITION);
    old_state(StateDefinition::Y_POSITION) = motion_primitive(trajectory_idx-1, StateDefinition::Y_POSITION);
    old_state(StateDefinition::THETA_POSITION) = motion_primitive(trajectory_idx-1, StateDefinition::THETA_POSITION);
    old_state(StateDefinition::VEL_POSITION) = motion_primitive(trajectory_idx-1, StateDefinition::VEL_POSITION);

    auto state = motion_primitive.row(trajectory_idx) = dynamic::euler_int(*dynamic_model_, old_state, dynamic_input, dt);
    motion_primitive(trajectory_idx, StateDefinition::TIME_POSITION) = trajectory_idx*dt;
    motion_primitive(trajectory_idx, StateDefinition::X_POSITION) = state(StateDefinition::X_POSITION);
    motion_primitive(trajectory_idx, StateDefinition::Y_POSITION) = state(StateDefinition::Y_POSITION);
    motion_primitive(trajectory_idx, StateDefinition::THETA_POSITION) = state(StateDefinition::THETA_POSITION);
    motion_primitive(trajectory_idx, StateDefinition::VEL_POSITION) = state(StateDefinition::VEL_POSITION);
  }
  // add remaining time of time span
  auto old_state = State(int(dynamic::StateDefinition::MIN_STATE_SIZE)); 
  old_state(StateDefinition::TIME_POSITION) = motion_primitive(trajectory_idx, StateDefinition::TIME_POSITION);
  old_state(StateDefinition::X_POSITION) = motion_primitive(trajectory_idx, StateDefinition::X_POSITION);
  old_state(StateDefinition::Y_POSITION) = motion_primitive(trajectory_idx, StateDefinition::Y_POSITION);
  old_state(StateDefinition::THETA_POSITION) = motion_primitive(trajectory_idx, StateDefinition::THETA_POSITION);
  old_state(StateDefinition::VEL_POSITION) = motion_primitive(trajectory_idx, StateDefinition::VEL_POSITION);

  trajectory_idx++;
  auto state = dynamic::euler_int(*dynamic_model_, old_state, dynamic_input, time_span-(num_trajectory_points-1)*dt);
  motion_primitive(trajectory_idx, StateDefinition::TIME_POSITION) = trajectory_idx*dt;
  motion_primitive(trajectory_idx, StateDefinition::X_POSITION) = state(StateDefinition::X_POSITION);
  motion_primitive(trajectory_idx, StateDefinition::Y_POSITION) = state(StateDefinition::Y_POSITION);
  motion_primitive(trajectory_idx, StateDefinition::THETA_POSITION) = state(StateDefinition::THETA_POSITION);
  motion_primitive(trajectory_idx, StateDefinition::VEL_POSITION) = state(StateDefinition::VEL_POSITION);

  motion_primitives_.push_back(motion_primitive);
  return motion_primitives_.size()-1;
}

void BehaviorMotionPrimitives::ActionToBehavior(const MotionIdx& motion_idx) {
  active_motion_ = motion_idx;
}

}  // namespace behavior
}  // namespace models
}  // namespace modules
