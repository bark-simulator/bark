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

  if ((world_time + 1e-4) < trajectory(0, TIME_POSITION) ||
      (world_time - 1e-4) > trajectory(trajectory.rows() - 1, TIME_POSITION)) {
    is_in_traj = false;
    LOG(INFO) << "World time " << world_time << " out of trajectory."
              << " Trajectory start_time: " << trajectory(0, TIME_POSITION)
              << ", end_time: " << trajectory(trajectory.rows() - 1, TIME_POSITION)
              << "." << std::endl;
    LOG(INFO) << trajectory << std::endl;
  }
  return is_in_traj;
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

  // TODO(@hart): 2. check if timepoint is exactly contained
  // std::pair<State, bool> CheckIfTimeIsInTrajectory(traj, time)
  // then set; done.

  // TODO(@hart): 3. if not interpolate
  // Interpolate(traj, time) <- could be linear, quadratic etc.

  int index_world_time = 0;
  float min_time_diff = std::numeric_limits<float>::max();
  for (int i = 0; i < trajectory.rows(); i++) {
    float diff_time = fabs(trajectory(i, dynamic::TIME_POSITION) - new_world_time);
    if (diff_time < min_time_diff) {
      index_world_time = i;
      min_time_diff = diff_time;
    }
  }

  // house-keeping
  SetLastTrajectory(trajectory);
  SetLastState(State(trajectory.row(index_world_time)));
}

}  // namespace execution
}  // namespace models
}  // namespace modules
