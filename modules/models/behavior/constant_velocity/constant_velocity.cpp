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

  const auto& lane_corr = observed_world.GetLaneCorridor();
  if (!lane_corr) {
    LOG(INFO) << "Agent " << observed_world.GetEgoAgentId()
              << ": Behavior status has expired!" << std::endl;
    SetBehaviorStatus(BehaviorStatus::EXPIRED);
    return GetLastTrajectory();
  }

  double dt = delta_time / (GetNumTrajectoryTimePoints() - 1);
  // interaction term off and GetTotalAcc returns const. acc.
  IDMRelativeValues rel_values{0., 0., false};
  std::tuple<Trajectory, Action> traj_action =
    GenerateTrajectory(
      observed_world, lane_corr, rel_values, dt);

  // set values
  Trajectory traj = std::get<0>(traj_action);
  Action action = std::get<1>(traj_action);
  SetLastTrajectory(traj);
  SetLastAction(action);
  return traj;
}

/**
 * @brief Constant acceleration
 * 
 * @return std::pair<double, double> acceleration, total_distance
 */
std::pair<double, double> BehaviorConstantVelocity::GetTotalAcc(
  const world::ObservedWorld& observed_world,
  const IDMRelativeValues& rel_values,
  double rel_distance,
  double dt) const {
  return std::pair<double, double>(0., 0.);
}

}  // namespace behavior
}  // namespace models
}  // namespace modules
