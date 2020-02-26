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

//! IDM Model will assume other front vehicle as constant velocity during
//! delta_time
Trajectory BehaviorIDMLaneTracking::Plan(
    float delta_time, const world::ObservedWorld& observed_world) {
  const DynamicModelPtr dynamic_model =
      observed_world.GetEgoAgent()->GetDynamicModel();
  auto single_track =
      std::dynamic_pointer_cast<dynamic::SingleTrackModel>(dynamic_model);
  if (!single_track) {
    LOG(FATAL) << "Only SingleTrack as dynamic model supported!";
  }

  std::pair<AgentPtr, FrenetPosition> leading_vehicle =
      observed_world.GetAgentInFront();
  std::shared_ptr<const Agent> ego_agent = observed_world.GetEgoAgent();

  using dynamic::StateDefinition;
  //! TODO(@fortiss): parameters
  const float min_velocity = GetMinVelocity();
  const float max_velocity = GetMaxVelocity();

  const int num_traj_time_points = 11;
  dynamic::Trajectory traj(num_traj_time_points,
                           static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  float const dt = delta_time / (num_traj_time_points - 1);

  dynamic::State ego_vehicle_state = observed_world.CurrentEgoState();

  // select state and get p0
  geometry::Point2d pose = observed_world.CurrentEgoPosition();

  auto lane_corr = observed_world.GetLaneCorridor();
  if (!lane_corr) {
    this->SetLastTrajectory(traj);
    return traj;
  }

  geometry::Line line = lane_corr->GetCenterLine();

  double net_distance = .0f;
  double vel_other = 1e6;
  if (leading_vehicle.first) {
    net_distance = CalcNetDistance(ego_agent, leading_vehicle.first);
    dynamic::State other_vehicle_state =
        leading_vehicle.first->GetCurrentState();
    vel_other = other_vehicle_state(StateDefinition::VEL_POSITION);
  }

  if (!line.obj_.empty()) {
    // adding state at t=0
    traj.block<1, StateDefinition::MIN_STATE_SIZE>(0, 0) =
        ego_vehicle_state.transpose().block<1, StateDefinition::MIN_STATE_SIZE>(
            0, 0);

    float vel_i = ego_vehicle_state(StateDefinition::VEL_POSITION);
    double acc;
    double traveled_ego;
    double traveled_other;

    for (int i = 1; i < num_traj_time_points; ++i) {
      if (leading_vehicle.first) {
        acc = CalcIDMAcc(net_distance, vel_i, vel_other);
        traveled_ego = +0.5f * acc * dt * dt + vel_i * dt;
        traveled_other = vel_other * dt;
        net_distance += traveled_other - traveled_ego;
      } else {
        acc = GetMaxAcceleration() * CalcFreeRoadTerm(vel_i);
      }

      BARK_EXPECT_TRUE(!std::isnan(acc));

      double angle = CalculateSteeringAngle(single_track, traj.row(i - 1), line,
                                            crosstrack_error_gain_);

      dynamic::Input input(2);
      input << acc, angle;
      traj.row(i) =
          dynamic::euler_int(*dynamic_model, traj.row(i - 1), input, dt);
      // Do not allow negative speeds
      traj(i, StateDefinition::VEL_POSITION) =
          std::max(traj(i, StateDefinition::VEL_POSITION), 0.0f);
    }

    SetLastAction(Action(Continuous1DAction(acc)));
  }
  this->SetLastTrajectory(traj);
  return traj;
}

}  // namespace behavior
}  // namespace models
}  // namespace modules
