// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include "modules/models/behavior/dynamic_model/dynamic_model.hpp"
#include "modules/models/dynamic/integration.hpp"
#include "modules/world/observed_world.hpp"

namespace modules {
namespace models {
namespace behavior {
using dynamic::StateDefinition::TIME_POSITION;
using dynamic::StateDefinition::X_POSITION;
using dynamic::StateDefinition::Y_POSITION;
using dynamic::StateDefinition::THETA_POSITION;
using dynamic::StateDefinition::VEL_POSITION;


DynamicBehaviorModel::DynamicBehaviorModel(const DynamicModelPtr& dynamic_model,
                           commons::Params *params) :
  BehaviorModel(params),
  dynamic_model_(dynamic_model),
  current_action_(2),
  integration_time_delta_(
    params->get_real("integration_time_delta",
                      "delta t for integration", 0.01)) {
    current_action_.setZero();
  }

DynamicBehaviorModel::DynamicBehaviorModel(DynamicBehaviorModel* other_behavior) :
  BehaviorModel(other_behavior->get_params()),
  dynamic_model_(other_behavior->dynamic_model_),
  integration_time_delta_(other_behavior->integration_time_delta_) {}

dynamic::Trajectory DynamicBehaviorModel::Plan(
    float delta_time,
    const world::ObservedWorld& observed_world) {

  dynamic::State ego_vehicle_state =
    observed_world.get_ego_agent()->get_current_state();

  std::cout << ego_vehicle_state << std::endl;
  double start_time = observed_world.get_world_time();
  float dt = integration_time_delta_;
  int num_trajectory_points = static_cast<int>(std::ceil(delta_time / dt));

  dynamic::Trajectory traj(
    num_trajectory_points,
    static_cast<int>(dynamic::StateDefinition::MIN_STATE_SIZE));

  // std::cout << num_trajectory_points << ", " << ego_vehicle_state << ", " << current_action_ << std::endl;
  traj.row(0) = ego_vehicle_state;
  for (int i = 1; i < num_trajectory_points; i++) {
    auto next_state = dynamic::euler_int(*dynamic_model_,
                                         traj.row(i-1),
                                         current_action_,
                                         dt);
    traj(i, TIME_POSITION) = start_time + i*dt;
    traj(i, X_POSITION) = next_state(X_POSITION);
    traj(i, Y_POSITION) = next_state(Y_POSITION);
    traj(i, THETA_POSITION) = next_state(THETA_POSITION);
    traj(i, VEL_POSITION) = next_state(VEL_POSITION);
  }
 
  // std::cout << "=====================" << std::endl;
  // std::cout << traj << std::endl;
  this->set_last_trajectory(traj);
  return traj;
}


}  // namespace behavior
}  // namespace models
}  // namespace modules
