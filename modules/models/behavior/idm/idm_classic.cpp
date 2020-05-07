// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/models/behavior/idm/idm_classic.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>
#include <tuple>

#include "modules/commons/transformation/frenet.hpp"
#include "modules/world/observed_world.hpp"

namespace modules {
namespace models {
namespace behavior {

using modules::commons::transformation::FrenetPosition;
using modules::geometry::Point2d;
using modules::models::dynamic::State;
using modules::models::dynamic::StateDefinition;
using modules::world::objects::Agent;
using modules::world::objects::AgentPtr;
using modules::world::map::LaneCorridor;
using modules::world::map::LaneCorridorPtr;


std::tuple<Trajectory, Action> BehaviorIDMClassic::GenerateTrajectory(
  const world::ObservedWorld& observed_world,
  const std::tuple<double, double, bool>& rel_values,
  float delta_time) const {
  // definitions
  double rel_distance = std::get<0>(rel_values);
  double vel_front = std::get<1>(rel_values);
  double interaction_term_active = std::get<2>(rel_values);
  auto lane_corr = observed_world.GetLaneCorridor();
  double t_i, acc, traveled_ego, traveled_other;
  geometry::Line line = lane_corr->GetCenterLine();
  // TODO(@hart): why 11
  const int num_traj_time_points = 11;
  dynamic::Trajectory traj(num_traj_time_points,
                           static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  float const dt = delta_time / (num_traj_time_points - 1);

  // calculate traj.
  dynamic::State ego_vehicle_state = observed_world.CurrentEgoState();
  // select state and get p0
  geometry::Point2d pose = observed_world.CurrentEgoPosition();
  if (!line.obj_.empty()) {
    // adding state at t=0
    traj.block<1, StateDefinition::MIN_STATE_SIZE>(0, 0) =
        ego_vehicle_state.transpose().block<1, StateDefinition::MIN_STATE_SIZE>(
            0, 0);

    float s_start = GetNearestS(line, pose);  // checked
    double start_time = observed_world.GetWorldTime();
    float vel_i = ego_vehicle_state(StateDefinition::VEL_POSITION);
    float s_i = s_start;

    for (int i = 1; i < num_traj_time_points; ++i) {
      if (interaction_term_active) {
        acc = CalcIDMAcc(rel_distance, vel_i, vel_front);
        traveled_ego = 0.5f * acc * dt * dt + vel_i * dt;
        traveled_other = vel_front * dt;
        rel_distance += traveled_other - traveled_ego;
      } else {
        acc = GetMaxAcceleration() * CalcFreeRoadTerm(vel_i);
      }

      BARK_EXPECT_TRUE(!std::isnan(acc));
      s_i += 0.5f * acc * dt * dt + vel_i * dt;
      const float temp_velocity = vel_i + acc * dt;
      vel_i = std::max(std::min(
        temp_velocity, GetMaxVelocity()), GetMinVelocity());
      t_i = static_cast<float>(i) * dt + start_time;
      geometry::Point2d traj_point = GetPointAtS(line, s_i);
      float traj_angle = GetTangentAngleAtS(line, s_i);

      BARK_EXPECT_TRUE(!std::isnan(boost::geometry::get<0>(traj_point)));
      BARK_EXPECT_TRUE(!std::isnan(boost::geometry::get<1>(traj_point)));
      BARK_EXPECT_TRUE(!std::isnan(traj_angle));

      traj(i, StateDefinition::TIME_POSITION) = t_i;
      traj(i, StateDefinition::X_POSITION) =
        boost::geometry::get<0>(traj_point);
      traj(i, StateDefinition::Y_POSITION) =
        boost::geometry::get<1>(traj_point);
      traj(i, StateDefinition::THETA_POSITION) = traj_angle;
      traj(i, StateDefinition::VEL_POSITION) = vel_i;
    }
  }

  Action action(acc);
  return std::tuple<Trajectory, Action>(traj, action);
}

//! IDM Model will assume other front vehicle as constant velocity during
Trajectory BehaviorIDMClassic::Plan(
    float delta_time, const world::ObservedWorld& observed_world) {
  using dynamic::StateDefinition;
  SetBehaviorStatus(BehaviorStatus::VALID);

  // TODO(@hart): could also be a different lane corridor
  //              for lane tracking
  auto lane_corr = observed_world.GetLaneCorridor();
  if (!lane_corr) {
    return GetLastTrajectory();
  }
  std::tuple<double, double, bool> rel_values = CalcRelativeValues(
    observed_world,
    lane_corr);
  double rel_distance = std::get<0>(rel_values);
  double vel_front = std::get<1>(rel_values);
  double interaction_term_active = std::get<2>(rel_values);

  std::tuple<Trajectory, Action> traj_action =
    GenerateTrajectory(observed_world, rel_values, delta_time);
  Trajectory traj = std::get<0>(traj_action);
  Action action = std::get<1>(traj_action);
  SetLastTrajectory(traj);
  SetLastAction(action);
  return traj;
}

}  // namespace behavior
}  // namespace models
}  // namespace modules
