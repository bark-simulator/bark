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
  const float delta = 1e-3;
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
  const float delta = 1e-3;
  float start_time = trajectory(0, TIME_POSITION);
  float end_time = trajectory(trajectory.rows() - 1, TIME_POSITION);

  // closer to the end; reverse
  if (fabs(world_time - end_time) < fabs(world_time - start_time)) {
    for (int i = trajectory.rows() - 1; 0 < i; i--)
      if ( fabs(trajectory(i, dynamic::TIME_POSITION) - world_time) < delta)
        return {State(trajectory.row(i)), true};
  } else {
    for (int i = 0; i < trajectory.rows(); i++)
      if ( fabs(trajectory(i, dynamic::TIME_POSITION) - world_time) < delta)
        return {State(trajectory.row(i)), true};
  }
  return {State(), false};
}

std::pair<int, bool> ExecutionModelInterpolate::FindClosestLowerTrajectoryRow(
  const Trajectory& trajectory,
  const double& world_time) const {
  const float delta = 1e-3;
  float start_time = trajectory(0, TIME_POSITION);
  float end_time = trajectory(trajectory.rows() - 1, TIME_POSITION);
  float min_traj_time_distance = std::numeric_limits<float>::max();

  int ret_idx = 0;
  bool found_closest_pt = false;
  for (int i = trajectory.rows() - 1; 0 < i; i--) {
    float time_dist = fabs(trajectory(i, dynamic::TIME_POSITION) - world_time);
    if (time_dist < min_traj_time_distance && trajectory(i, dynamic::TIME_POSITION) < world_time) {
      min_traj_time_distance = time_dist;
      ret_idx = i;
      found_closest_pt = true;
    }
  }
  return {ret_idx, found_closest_pt};
}

State ExecutionModelInterpolate::Interpoalte(
  const State& p0, const State& p1, const float& time) const {
  const float start_time = p0(dynamic::TIME_POSITION);
  const float end_time = p1(dynamic::TIME_POSITION);
  const float lambda = fabs((time - start_time) / (end_time - start_time));
  // LOG(INFO) << lambda << ", time:" << time << ", " << start_time << ", " << end_time << std::endl;
  return lambda*p0  + (1 - lambda)*p1;
}

void ExecutionModelInterpolate::Execute(
  const float& new_world_time,
  const Trajectory& trajectory,
  const DynamicModelPtr dynamic_model) {

  // TODO(@hart): for this model not required
  SetLastTrajectory(trajectory);
  
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
    return;
  }

  // Interpolate(traj, time) <- could be linear, quadratic etc.
  std::pair<int, bool> lower_idx = FindClosestLowerTrajectoryRow(
    trajectory, new_world_time);
  
  if (lower_idx.second == true) {
    int lower_id = lower_idx.first;
    int upper_id = lower_id + 1;
    State p0 = trajectory.row(lower_id);
    State p1 = trajectory.row(upper_id);
    // LOG(INFO) << "Interpolating" << std::endl;
    State interp_state = Interpoalte(p0, p1, new_world_time);
    SetLastState(interp_state);
    // LOG(INFO) << "Interpoalted time: " << interp_state(dynamic::TIME_POSITION) << std::endl;
    // TODO(@hart): check time for concurrency
    return;
  } else {
    SetExecutionStatus(ExecutionStatus::INVALID);
  }
}

}  // namespace execution
}  // namespace models
}  // namespace modules
