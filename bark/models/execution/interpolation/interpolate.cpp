// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/models/execution/interpolation/interpolate.hpp"
#include <iostream>

namespace bark {
namespace models {
namespace execution {

using dynamic::DynamicModelPtr;
using dynamic::Trajectory;
using dynamic::StateDefinition::TIME_POSITION;

bool ExecutionModelInterpolate::CheckIfWorldTimeIsWithinTrajectory(
    const Trajectory& trajectory, const float& world_time) const {
  bool is_in_traj = true;
  const float delta = 1e-3;
  if ((world_time + delta) < trajectory(0, TIME_POSITION) ||
      (world_time - delta) > trajectory(trajectory.rows() - 1, TIME_POSITION)) {
    is_in_traj = false;
    LOG(INFO) << "World time " << world_time << " out of trajectory."
              << " Trajectory start_time: " << trajectory(0, TIME_POSITION)
              << ", end_time: "
              << trajectory(trajectory.rows() - 1, TIME_POSITION) << "."
              << std::endl;
    LOG(INFO) << trajectory << std::endl;
  }
  return is_in_traj;
}

std::pair<State, bool>
ExecutionModelInterpolate::CheckIfTimeExactIsInTrajectory(
    const Trajectory& trajectory, const double& world_time) const {
  const float delta = 1e-3;
  float start_time = trajectory(0, TIME_POSITION);
  float end_time = trajectory(trajectory.rows() - 1, TIME_POSITION);
  int half_traj_num = static_cast<int>(trajectory.rows() / 2);

  // closer to the end; reverse; only need to check half of the traj.
  if (fabs(world_time - end_time) < fabs(world_time - start_time)) {
    for (int i = trajectory.rows() - 1; half_traj_num - 1 < i; i--)
      if (fabs(trajectory(i, TIME_POSITION) - world_time) < delta)
        return {State(trajectory.row(i)), true};
  } else {
    for (int i = 0; i < half_traj_num + 1; i++)
      if (fabs(trajectory(i, TIME_POSITION) - world_time) < delta)
        return {State(trajectory.row(i)), true};
  }
  return {State(), false};
}

std::pair<int, bool> ExecutionModelInterpolate::FindClosestLowerTrajectoryRow(
    const Trajectory& trajectory, const double& world_time) const {
  const float delta = 1e-3;
  int ret_idx = 0;
  bool found_closest_pt = false;
  for (int i = 0; i < trajectory.rows(); i++) {
    if (world_time >= trajectory(i, TIME_POSITION)) {
      ret_idx = i;
      found_closest_pt = true;
      break;
    }
  }
  return {ret_idx, found_closest_pt};
}

State ExecutionModelInterpolate::Interpolate(const State& p0, const State& p1,
                                             const float& time) const {
  const float start_time = p0(TIME_POSITION);
  const float end_time = p1(TIME_POSITION);
  const float lambda = fabs((time - start_time) / (end_time - start_time));
  BARK_EXPECT_TRUE(end_time >= start_time && time >= start_time);
  return (1 - lambda) * p0 + (lambda)*p1;
}

void ExecutionModelInterpolate::Execute(const float& new_world_time,
                                        const Trajectory& trajectory,
                                        const DynamicModelPtr dynamic_model) {
  // book-keeping
  SetLastTrajectory(trajectory);

  // check time and size
  if (!CheckIfWorldTimeIsWithinTrajectory(trajectory, new_world_time)) {
    SetExecutionStatus(ExecutionStatus::INVALID);
    return;
  } else {
    SetExecutionStatus(ExecutionStatus::VALID);
  }

  // check if we can quickly find an exact point
  std::pair<State, bool> has_exact_point =
      CheckIfTimeExactIsInTrajectory(trajectory, new_world_time);
  if (has_exact_point.second) {
    SetLastState(has_exact_point.first);
    return;
  }

  // if not, interpolate
  std::pair<int, bool> lower_idx =
      FindClosestLowerTrajectoryRow(trajectory, new_world_time);
  if (lower_idx.second == true) {
    int lower_id = lower_idx.first;
    int upper_id = lower_id + 1;
    State p0 = trajectory.row(lower_id);
    State p1 = trajectory.row(upper_id);
    State interp_state = Interpolate(p0, p1, new_world_time);
    SetLastState(interp_state);
    // assert that the interpolated point is near the world time
    BARK_EXPECT_TRUE(fabs(interp_state(TIME_POSITION) - new_world_time) < 0.02);
    return;
  } else {
    LOG(INFO) << "ExecutionStatus is invalid." << std::endl;
    SetExecutionStatus(ExecutionStatus::INVALID);
  }
}

}  // namespace execution
}  // namespace models
}  // namespace bark
