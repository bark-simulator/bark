// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/models/behavior/dynamic_model/dynamic_model.hpp"
#include <cmath>
#include "bark/models/dynamic/integration.hpp"
#include "bark/models/dynamic/single_track.hpp"
#include "bark/world/observed_world.hpp"

namespace bark {
namespace models {
namespace behavior {

using dynamic::Input;
using dynamic::SingleTrackModel;
using dynamic::StateDefinition;
using dynamic::StateDefinition::THETA_POSITION;
using dynamic::StateDefinition::TIME_POSITION;
using dynamic::StateDefinition::VEL_POSITION;
using dynamic::StateDefinition::X_POSITION;
using dynamic::StateDefinition::Y_POSITION;

BehaviorDynamicModel::BehaviorDynamicModel(const commons::ParamsPtr& params)
    : BehaviorModel(params),
      integration_time_delta_(
          params->GetReal("BehaviorDynamicModel::IntegrationTimeDelta",
                          "delta t for integration", 0.05)) {}

dynamic::Trajectory BehaviorDynamicModel::Plan(
    float min_planning_time, const world::ObservedWorld& observed_world) {
  SetBehaviorStatus(BehaviorStatus::VALID);
  const DynamicModelPtr dynamic_model =
      observed_world.GetEgoAgent()->GetDynamicModel();
  auto single_track =
      std::dynamic_pointer_cast<SingleTrackModel>(dynamic_model);
  if (!single_track)
    LOG(FATAL) << "Only SingleTrack as dynamic model supported!";

  dynamic::State ego_vehicle_state =
      observed_world.GetEgoAgent()->GetCurrentState();
  double start_time = observed_world.GetWorldTime();
  float dt = integration_time_delta_;
  int num_trajectory_points =
      static_cast<int>(std::ceil(min_planning_time / dt)) + 1;

  dynamic::Trajectory traj(num_trajectory_points,
                           static_cast<int>(StateDefinition::MIN_STATE_SIZE));

  // this action is set externally. e.g. by RL
  Input action = boost::get<Input>(action_);

  // generate a trajectory with const. action
  traj.row(0) = ego_vehicle_state;
  for (int i = 1; i < num_trajectory_points; i++) {
    auto next_state =
        dynamic::euler_int(*dynamic_model, traj.row(i - 1), action, dt);
    traj.row(i) = next_state;
    traj(i, 0) = start_time + i * dt;
  }

  this->SetLastTrajectory(traj);
  return traj;
}

}  // namespace behavior
}  // namespace models
}  // namespace bark
