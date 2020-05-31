// Copyright (c) 2020 Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
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
  const LaneCorridorPtr& lane_corr,
  const IDMRelativeValues& rel_values,
  double dt) const {
  double t_i = 0., acc = 0.;
  geometry::Line line = lane_corr->GetCenterLine();
  dynamic::Trajectory traj(GetNumTrajectoryTimePoints(),
                           static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  dynamic::State ego_vehicle_state = observed_world.CurrentEgoState();
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

    double rel_distance = rel_values.leading_distance;
    // calc. traj.
    for (int i = 1; i < GetNumTrajectoryTimePoints(); ++i) {
      std::tie(acc, rel_distance) =
        GetTotalAcc(observed_world, rel_values, rel_distance, dt);
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

}  // namespace behavior
}  // namespace models
}  // namespace modules
