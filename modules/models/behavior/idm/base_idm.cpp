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



BaseIDM::BaseIDM(
  const commons::ParamsPtr& params) : BehaviorModel(params) {
  param_minimum_spacing_ = params->GetReal(
    "BehaviorIDMClassic::MinimumSpacing", "See Wikipedia IDM article", 2.0f);
  param_desired_time_head_way_ = params->GetReal(
    "BehaviorIDMClassic::DesiredTimeHeadway",
    "See Wikipedia IDM article", 1.5f);
  param_max_acceleration_ = params->GetReal(
    "BehaviorIDMClassic::MaxAcceleration", "See Wikipedia IDM article", 1.7f);
  param_acceleration_lower_bound_ = params->GetReal(
    "BehaviorIDMClassic::AccelerationLowerBound",
    "See Wikipedia IDM article", -5.0f);
  param_acceleration_upper_bound_ = params->GetReal(
    "BehaviorIDMClassic::AccelerationUpperBound",
    "See Wikipedia IDM article", 8.0f);
  param_desired_velocity_ = params->GetReal(
    "BehaviorIDMClassic::DesiredVelocity", "See Wikipedia IDM article", 15.0f);
  param_comfortable_braking_acceleration_ = params->GetReal(
    "BehaviorIDMClassic::ComfortableBrakingAcceleration",
    "See Wikipedia IDM article", 1.67f);
  param_min_velocity_ = params->GetReal(
    "BehaviorIDMClassic::MinVelocity", "See Wikipedia IDM article", 0.0f);
  param_max_velocity_ = params->GetReal(
    "BehaviorIDMClassic::MaxVelocity", "See Wikipedia IDM article", 50.0f);
  param_exponent_ = params->GetInt("BehaviorIDMClassic::Exponent",
  "See Wikipedia IDM article", 4);
  brake_lane_end_ = params->GetBool(
    "BehaviorIDMClassic::BrakeForLaneEnd",
    "Whether the vehicle should stop at the end of its LaneCorridor.",
    false);
  brake_lane_end_enabled_distance_ = params->GetReal(
    "BehaviorIDMClassic::BrakeForLaneEndEnabledDistance",
    "Range in m when the braking should be active",
    150);
  brake_lane_end_distance_offset_ = params->GetReal(
    "BehaviorIDMClassic::BrakeForLaneEndDistanceOffset",
    "Distance offset for vehicle to stop at.",
    20);
}

double BaseIDM::CalcFreeRoadTerm(const double vel_ego) const {
  const float desired_velocity = GetDesiredVelocity();
  const int exponent = GetExponent();
  double free_road_term = 1 - pow(vel_ego / desired_velocity, exponent);
  return free_road_term;
}

double BaseIDM::CalcInteractionTerm(double net_distance,
                                               double vel_ego,
                                               double vel_other) const {
  // Parameters
  const float minimum_spacing = GetMinimumSpacing();
  const float desired_time_headway = GetDesiredTimeHeadway();
  const float max_acceleration = GetMaxAcceleration();
  const float comfortable_braking_acceleration =
    GetComfortableBrakingAcceleration();
  net_distance = std::max(net_distance, 0.0);
  const double net_velocity = vel_ego - vel_other;
  const double helper_state =
    minimum_spacing + vel_ego * desired_time_headway +
    (vel_ego * net_velocity) /
    (2 * sqrt(max_acceleration * comfortable_braking_acceleration));
  BARK_EXPECT_TRUE(!std::isnan(helper_state));
  double interaction_term =
    (helper_state / net_distance) * (helper_state / net_distance);
  if (std::isnan(interaction_term)) {
    interaction_term = std::numeric_limits<double>::infinity();
  }
  return interaction_term;
}

double BaseIDM::CalcNetDistance(
    const std::shared_ptr<const Agent>& ego_agent,
    const std::shared_ptr<const Agent>& leading_agent) const {
  // relative velocity and longitudinal distance
  const State ego_state = ego_agent->GetCurrentState();
  FrenetPosition frenet_ego = ego_agent->CurrentFrenetPosition();
  const float ego_velocity = ego_state(StateDefinition::VEL_POSITION);

  // Leading vehicle exists in driving corridor, we calculate interaction term
  const State leading_state = leading_agent->GetCurrentState();
  const float other_velocity = leading_state(StateDefinition::VEL_POSITION);

  FrenetPosition frenet_leading = leading_agent->CurrentFrenetPosition();
  const float vehicle_length =
      ego_agent->GetShape().front_dist_ + leading_agent->GetShape().rear_dist_;
  const double net_distance =
      frenet_leading.lon - vehicle_length - frenet_ego.lon;
  return net_distance;
}

double BaseIDM::CalcIDMAcc(const double net_distance,
                                      const double vel_ego,
                                      const double vel_other) const {
  const float acc_lower_bound = GetAccelerationLowerBound();
  const float acc_upper_bound = GetAccelerationUpperBound();
  const float max_acceleration = GetMaxAcceleration();
  const float free_road_term = CalcFreeRoadTerm(vel_ego);
  const float interaction_term =
    CalcInteractionTerm(net_distance, vel_ego, vel_other);
  float acc = max_acceleration * (free_road_term - interaction_term);
  // For now, linit acceleration of IDM to brake with -acc_max
  acc = std::max(std::min(acc, acc_upper_bound), acc_lower_bound);
  return acc;
}


std::tuple<double, double, bool> BaseIDM::CalcRelativeValues(
  const world::ObservedWorld& observed_world,
  const LaneCorridorPtr& lane_corr) const {
  bool interaction_term_active = false;
  double leading_distance = 0.;
  double leading_velocity = 1e6;

  // TODO(@hart): should be based on the lane_corr
  std::pair<AgentPtr, FrenetPosition> leading_vehicle =
    observed_world.GetAgentInFront();
  std::shared_ptr<const Agent> ego_agent = observed_world.GetEgoAgent();

  // vehicles
  if (leading_vehicle.first) {
    leading_distance = CalcNetDistance(ego_agent, leading_vehicle.first);
    dynamic::State other_vehicle_state =
      leading_vehicle.first->GetCurrentState();
    leading_velocity = other_vehicle_state(StateDefinition::VEL_POSITION);
    interaction_term_active = true;
  }

  // 2nd part for lane_corr end
  if (brake_lane_end_) {
    const double len_until_end =
      lane_corr->LengthUntilEnd(observed_world.CurrentEgoPosition())
      - brake_lane_end_distance_offset_;
    if (len_until_end < brake_lane_end_enabled_distance_)
      interaction_term_active = true;
    // if no leading vehicle
    if (!leading_vehicle.first &&
        len_until_end < brake_lane_end_enabled_distance_) {
      leading_distance = len_until_end;
      leading_velocity = 0.;
    // if there is a leading vehicle
    } else if (len_until_end < brake_lane_end_enabled_distance_) {
      leading_distance = std::min(leading_distance, len_until_end);
      // lane end has 0 vel.
      if (leading_distance == len_until_end)
        leading_velocity = 0.;
    }
  }

  return std::make_tuple(
    leading_distance,
    leading_velocity,
    interaction_term_active);
}

double BaseIDM::CalcRawIDMAcc(const double& net_distance,
                                         const double& vel_ego,
                                         const double& vel_other) const {
  const double free_road_term = CalcFreeRoadTerm(vel_ego);
  const double interaction_term =
    CalcInteractionTerm(net_distance, vel_ego, vel_other);
  return GetMaxAcceleration() * (free_road_term - interaction_term);
}

std::tuple<Trajectory, Action> BaseIDM::GenerateTrajectory(
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
//! delta_time
Trajectory BaseIDM::Plan(
    float delta_time, const world::ObservedWorld& observed_world) {
  using dynamic::StateDefinition;
  SetBehaviorStatus(BehaviorStatus::VALID);

  // TODO(@hart): could also be a different lane corridor
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
