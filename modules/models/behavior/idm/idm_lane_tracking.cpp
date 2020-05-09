// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/models/behavior/idm/idm_lane_tracking.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>
#include <tuple>

#include "modules/commons/transformation/frenet.hpp"
#include "modules/world/observed_world.hpp"
#include "modules/models/dynamic/single_track.hpp"
#include "modules/models/dynamic/integration.hpp"

namespace modules {
namespace models {
namespace behavior {

using modules::commons::transformation::FrenetPosition;
using modules::geometry::Point2d;
using modules::models::dynamic::State;
using modules::models::dynamic::StateDefinition;
using modules::world::objects::Agent;
using modules::world::objects::AgentPtr;
using modules::models::dynamic::DynamicModelPtr;


std::tuple<Trajectory, Action> BehaviorIDMLaneTracking::GenerateTrajectory(
    const world::ObservedWorld& observed_world,
    const LaneCorridorPtr& lane_corr,
    const std::tuple<double, double, bool>& rel_values,
    float delta_time) const {
  // definitions
  const DynamicModelPtr dynamic_model =
    observed_world.GetEgoAgent()->GetDynamicModel();
  auto single_track =
    std::dynamic_pointer_cast<dynamic::SingleTrackModel>(dynamic_model);
  if (!single_track) {
    LOG(FATAL) << "Only SingleTrack as dynamic model supported!";
  }
  dynamic::State ego_vehicle_state = observed_world.CurrentEgoState();
  double rel_distance = std::get<0>(rel_values);
  double vel_front = std::get<1>(rel_values);
  bool interaction_term_active = std::get<2>(rel_values);
  double t_i, acc, traveled_ego, traveled_other;
  geometry::Line line = lane_corr->GetCenterLine();
  // TODO(@hart): why 11
  const int num_traj_time_points = 11;
  dynamic::Trajectory traj(num_traj_time_points,
                           static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  float const dt = delta_time / (num_traj_time_points - 1);

  if (!line.obj_.empty()) {
    // adding state at t=0
    traj.block<1, StateDefinition::MIN_STATE_SIZE>(0, 0) =
      ego_vehicle_state.transpose().block<1, StateDefinition::MIN_STATE_SIZE>(
        0, 0);
    float vel_i = ego_vehicle_state(StateDefinition::VEL_POSITION);

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
      double angle = CalculateSteeringAngle(
        single_track, traj.row(i - 1), line, crosstrack_error_gain_,
        limit_steering_rate_);

      dynamic::Input input(2);
      input << acc, angle;
      traj.row(i) =
        dynamic::euler_int(*dynamic_model, traj.row(i - 1), input, dt);
      // Do not allow negative speeds
      traj(i, StateDefinition::VEL_POSITION) =
          std::max(traj(i, StateDefinition::VEL_POSITION), 0.0f);
    }
  }
  Action action(acc);
  return std::tuple<Trajectory, Action>(traj, action);
}

//! IDM Model will assume other front vehicle as constant velocity during
Trajectory BehaviorIDMLaneTracking::Plan(
    float delta_time, const world::ObservedWorld& observed_world) {
  using dynamic::StateDefinition;
  SetBehaviorStatus(BehaviorStatus::VALID);

  if (lane_corr_id_ != -1) {
    const auto& road_corr = observed_world.GetRoadCorridor();
    SetLaneCorridor(
      road_corr->GetUniqueLaneCorridors()[lane_corr_id_]);
  } else {
    SetLaneCorridor(observed_world.GetLaneCorridor());
  }

  if (!GetLaneCorridor()) {
    return GetLastTrajectory();
  }

  std::tuple<double, double, bool> rel_values = CalcRelativeValues(
    observed_world,
    GetLaneCorridor());


  std::tuple<Trajectory, Action> traj_action =
    GenerateTrajectory(
      observed_world, GetLaneCorridor(), rel_values, delta_time);

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
