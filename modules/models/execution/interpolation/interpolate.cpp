// Copyright (c) 2020 Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include <iostream>
#include "modules/models/execution/interpolation/interpolate.hpp"


namespace modules {
namespace models {
namespace execution {

using dynamic::Trajectory;
using dynamic::DynamicModelPtr;
using dynamic::TIME_POSITION;

bool ExecutionModelInterpolate::CheckIfWorldTimeIsWithinTrajectory(
  const Trajectory& trajectory,
  const float& world_time) const {
  bool is_in_traj = true;
  float delta = 1e-4;
  if ((world_time + delta) < trajectory(0, TIME_POSITION) ||
      (world_time - delta) > trajectory(trajectory.rows() - 1, TIME_POSITION)) {
    is_in_traj = false;
    LOG(INFO) << "World time " << world_time << " out of trajectory."
              << " Trajectory start_time: " << trajectory(0, TIME_POSITION)
              << ", end_time: " << trajectory(trajectory.rows() - 1, TIME_POSITION)
              << "." << std::endl;
    LOG(INFO) << trajectory << std::endl;
  }
  return is_in_traj;
}

std::pair<State, bool> ExecutionModelInterpolate::CheckIfTimeExactIsInTrajectory(
  const Trajectory& trajectory,
  const double& world_time) const {
  
  double start_time = trajectory(0, TIME_POSITION);
  double end_time = trajectory(trajectory.rows() - 1, TIME_POSITION);

  // closer to the end; reverse
  if (fabs(world_time - end_time) < fabs(world_time - start_time)) {
    for (int i = trajectory.rows() - 1; 0 < i; i--)
      if ( fabs(trajectory(i, dynamic::TIME_POSITION) - world_time) < 1e-4)
        return {State(trajectory.row(i)), true};
  } else {
    for (int i = 0; i < trajectory.rows(); i++)
      if ( fabs(trajectory(i, dynamic::TIME_POSITION) - world_time) < 1e-4)
        return {State(trajectory.row(i)), true};
  }
  return {State(), false};
}

void ExecutionModelInterpolate::Execute(
  const float& new_world_time,
  const Trajectory& trajectory,
  const DynamicModelPtr dynamic_model) {

  // check time and size
  if (!CheckIfWorldTimeIsWithinTrajectory(trajectory, new_world_time)) {
    SetExecutionStatus(ExecutionStatus::INVALID);
    return;
  } else {
    SetExecutionStatus(ExecutionStatus::VALID);
  }

  std::pair<State, bool> has_exact_point =
    CheckIfTimeExactIsInTrajectory(trajectory, new_world_time);
  if (has_exact_point.second) {
    SetLastState(has_exact_point.first);
  }

  // TODO(@hart): 3. if not interpolate
  // Interpolate(traj, time) <- could be linear, quadratic etc.

  // int index_world_time = 0;
  // float min_time_diff = std::numeric_limits<float>::max();
  // for (int i = 0; i < trajectory.rows(); i++) {
  //   float diff_time = fabs(trajectory(i, dynamic::TIME_POSITION) - new_world_time);
  //   if (diff_time < min_time_diff) {
  //     index_world_time = i;
  //     min_time_diff = diff_time;
  //   }
  // }

  // house-keeping
  SetLastTrajectory(trajectory);
  // SetLastState(State(trajectory.row(index_world_time)));
}

}  // namespace execution
}  // namespace models
}  // namespace modules
