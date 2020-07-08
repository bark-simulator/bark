// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <memory>

#include "bark/commons/transformation/frenet_state.hpp"
#include "bark/world/observed_world.hpp"
#include "behavior_static_trajectory.hpp"

namespace bark {
namespace models {
namespace behavior {

using bark::models::dynamic::StateDefinition;

BehaviorStaticTrajectory::BehaviorStaticTrajectory(
    const commons::ParamsPtr& params)
    : BehaviorModel(params, BehaviorStatus::NOT_STARTED_YET),
      static_trajectory_(ReadInStaticTrajectory(params->GetListListFloat(
          "static_trajectory",
          "List of states that form a static trajectory to follow", {{}}))) {
  SetLastAction(LonLatAction{0.0f, 0.0f});
}

BehaviorStaticTrajectory::BehaviorStaticTrajectory(
    const commons::ParamsPtr& params, const Trajectory& static_trajectory)
    : BehaviorModel(params, BehaviorStatus::NOT_STARTED_YET),
      static_trajectory_(static_trajectory) {
  SetLastAction(LonLatAction{0.0f, 0.0f});
}

Trajectory BehaviorStaticTrajectory::Plan(
    float delta_time, const bark::world::ObservedWorld& observed_world) {
  UpdateBehaviorStatus(delta_time, observed_world);

  const double start_time = observed_world.GetWorldTime();
  const double end_time = start_time + delta_time;

  StateRowVector interp_start;
  StateRowVector interp_end;
  int idx_start = Interpolate(start_time, &interp_start).second;
  int idx_end = Interpolate(end_time, &interp_end).first;

  if (idx_start < 0 || idx_end < 0) {
    auto traj = dynamic::Trajectory();
    this->SetLastAction(LonLatAction{0.0f, 0.0f});
    this->SetLastTrajectory(traj);
    return traj;
  }
  const int num_rows = idx_end - idx_start + 1;
  dynamic::Trajectory traj(
      num_rows + 2, static_cast<int>(dynamic::StateDefinition::MIN_STATE_SIZE));
  traj.row(0) = interp_start;
  traj.row(traj.rows() - 1) = interp_end;
  traj.block(1, 0, num_rows, traj.cols()) =
      static_trajectory_.block(idx_start, 0, num_rows, traj.cols());
  this->SetLastTrajectory(traj);
  this->SetLastAction(BehaviorStaticTrajectory::CalculateAction(
      delta_time, observed_world, traj));
  return traj;
}

Action BehaviorStaticTrajectory::CalculateAction(
    float delta_time, const bark::world::ObservedWorld& observed_world,
    const dynamic::Trajectory& trajectory) {
  auto lane_corridor = observed_world.GetLaneCorridor();
  BARK_EXPECT_TRUE(bool(lane_corridor));

  auto center_line = lane_corridor->GetCenterLine();
  bark::commons::transformation::FrenetState frenet_state_start(
      trajectory.row(0), center_line);
  bark::commons::transformation::FrenetState frenet_state_end(
      trajectory.row(trajectory.rows() - 1), center_line);
  auto acc_lat = (frenet_state_end.vlat - frenet_state_start.vlat) / delta_time;
  auto acc_lon = (frenet_state_end.vlon - frenet_state_start.vlon) / delta_time;

  return LonLatAction{acc_lat, acc_lon};
}

std::pair<int, int> BehaviorStaticTrajectory::Interpolate(
    const double t, StateRowVector* interpolated) const {
  StateRowVector delta;
  double alpha;
  int idx = -1;
  if (static_trajectory_.rows() < 2) {
    return {-1, -1};
  }
  for (int i = 0; i < static_trajectory_.rows() - 1; ++i) {
    float t_i = static_trajectory_(i, dynamic::TIME_POSITION);
    float t_i_succ = static_trajectory_(i + 1, dynamic::TIME_POSITION);

    if (t_i <= t && t <= t_i_succ) {
      idx = i;
      break;
    }
  }
  if (idx < 0) {
    return {-1, -1};
  }
  delta = static_trajectory_.row(idx + 1) - static_trajectory_.row(idx);
  alpha = (t - static_trajectory_(idx, dynamic::TIME_POSITION)) /
          delta(dynamic::TIME_POSITION);
  *interpolated = (static_trajectory_.row(idx) + alpha * delta);
  // Index of next valid entry
  if (alpha == 0.0) {
    return {idx - 1, idx + 1};
  } else if (alpha == 1.0) {
    return {idx, idx + 2};
  } else {
    return {idx, idx + 1};
  }
}

std::shared_ptr<BehaviorModel> BehaviorStaticTrajectory::Clone() const {
  std::shared_ptr<BehaviorStaticTrajectory> model_ptr =
      std::make_shared<BehaviorStaticTrajectory>(*this);
  return std::dynamic_pointer_cast<BehaviorModel>(model_ptr);
}

Trajectory BehaviorStaticTrajectory::ReadInStaticTrajectory(
    std::vector<std::vector<float>> list) {
  Trajectory traj(list.size(), list[0].size());
  for (int i = 0; i < traj.rows(); ++i) {
    BARK_EXPECT_TRUE(list[i].size() == static_cast<size_t>(traj.cols()));
    for (int j = 0; j < traj.cols(); ++j) {
      traj(i, j) = list[i][j];
    }
  }
  return traj;
}

const Trajectory& BehaviorStaticTrajectory::GetStaticTrajectory() const {
  return static_trajectory_;
}

void BehaviorStaticTrajectory::UpdateBehaviorStatus(
    float delta_time, const bark::world::ObservedWorld& observed_world) {
  const double start_time = observed_world.GetWorldTime();
  const double end_time = start_time + delta_time;

  const double start_time_static_traj =
      (static_trajectory_.col(dynamic::TIME_POSITION)).minCoeff();
  const double end_time_static_traj =
      (static_trajectory_.col(dynamic::TIME_POSITION)).maxCoeff();

  if (start_time_static_traj > start_time) {
    SetBehaviorStatus(BehaviorStatus::NOT_STARTED_YET);
  } else if (end_time_static_traj <= end_time) {
    VLOG(1) << "Agent " << observed_world.GetEgoAgentId()
            << ": Behavior status has expired!" << std::endl;
    SetBehaviorStatus(BehaviorStatus::EXPIRED);
  } else {
    SetBehaviorStatus(BehaviorStatus::VALID);
  }
}

}  // namespace behavior
}  // namespace models
}  // namespace bark
