// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <cmath>
#include "modules/models/behavior/dynamic_model/dynamic_model.hpp"
#include "modules/models/dynamic/integration.hpp"
#include "modules/models/dynamic/triple_integrator.hpp"
#include "modules/world/observed_world.hpp"

namespace modules {
namespace models {
namespace behavior {
using dynamic::StateDefinition::TIME_POSITION;
using dynamic::StateDefinition::X_POSITION;
using dynamic::StateDefinition::Y_POSITION;
using dynamic::StateDefinition::THETA_POSITION;
using dynamic::StateDefinition::VEL_POSITION;
using dynamic::TripleIntegratorModel;
using dynamic::Input;


DynamicBehaviorModel::DynamicBehaviorModel(
  const DynamicModelPtr& dynamic_model,
  const commons::ParamsPtr& params) :
  BehaviorModel(params),
  dynamic_model_(dynamic_model),
  integration_time_delta_(
    params->GetReal("integration_time_delta",
                      "delta t for integration", 0.01)) {
  }

DynamicBehaviorModel::DynamicBehaviorModel(DynamicBehaviorModel* other_behavior) :
  BehaviorModel(other_behavior->GetParams()),
  dynamic_model_(other_behavior->dynamic_model_),
  integration_time_delta_(other_behavior->integration_time_delta_) {}

dynamic::Trajectory DynamicBehaviorModel::Plan(
    float delta_time,
    const world::ObservedWorld& observed_world) {

  dynamic::State ego_vehicle_state =
    observed_world.GetEgoAgent()->GetCurrentState();

  double start_time = observed_world.GetWorldTime();
  float dt = integration_time_delta_;
  int num_trajectory_points = static_cast<int>(std::ceil(delta_time / dt));

  dynamic::Trajectory traj(
    num_trajectory_points,
    this->GetParams()->GetInt("DynamicModel::state_dimension",
                              "state vector length", 5));

  // std::cout << "State:" << ego_vehicle_state << std::endl;
  Input action = boost::get<Input>(
    this->GetLastAction());
  // std::cout << "Action:" << action << std::endl;

  traj.row(0) = ego_vehicle_state;
  for (int i = 1; i < num_trajectory_points; i++) {
    auto next_state = dynamic::euler_int(
      *dynamic_model_,
      traj.row(i-1),
      action,
      dt);
    traj.row(i) = next_state;
    traj(i, 0) = start_time + i*dt;
    if (std::dynamic_pointer_cast<TripleIntegratorModel>(dynamic_model_)) {
      traj(i, 1) = next_state(6);
      traj(i, 2) = next_state(9);
      traj(i, 3) = atan2(next_state(10), next_state(7));
      traj(i, 4) = sqrt(pow(next_state(7), 2) + pow(next_state(10), 2));
    }
  }

  // std::cout << "=====================" << std::endl;
  // std::cout << traj << std::endl;
  this->SetLastTrajectory(traj);
  return traj;
}


}  // namespace behavior
}  // namespace models
}  // namespace modules
