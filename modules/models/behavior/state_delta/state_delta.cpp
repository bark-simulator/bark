// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include "modules/models/behavior/state_delta/state_delta.hpp"
#include "modules/world/observed_world.hpp"

namespace modules {
namespace models {

dynamic::Trajectory behavior::BehaviorStateDelta::Plan(
    float delta_time,
    const world::ObservedWorld& observed_world) {

  using namespace dynamic;

  //! TODO(@fortiss): parameters
  const int num_traj_time_points = 2;
  dynamic::Trajectory traj(num_traj_time_points, int(StateDefinition::MIN_STATE_SIZE));

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

  this->set_last_trajectory(traj);
  return traj;
}

}  // namespace models
}  // namespace modules
