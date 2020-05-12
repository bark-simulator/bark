// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle,
// Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <tuple>

#include "modules/models/behavior/constant_velocity/constant_velocity.hpp"
#include "modules/world/observed_world.hpp"

namespace modules {
namespace models {
namespace behavior {

Trajectory BehaviorConstantVelocity::Plan(
    float delta_time, const world::ObservedWorld& observed_world) {
  SetBehaviorStatus(BehaviorStatus::VALID);

  if (!GetLaneCorridor()) {
    return GetLastTrajectory();
  }

  double dt = delta_time / (GetNumTrajectoryTimePoints() - 1);
  double acc = 0.;

  std::tuple<Trajectory, Action> traj_action =
    GenerateTrajectory(
      observed_world, GetLaneCorridor(), acc, dt);

  // set values
  Trajectory traj = std::get<0>(traj_action);
  Action action = std::get<1>(traj_action);
  SetLastTrajectory(traj);
  SetLastAction(action);
  return traj;
}

}  // namespace behavior
}  // namespace models
}  // namespace modules
