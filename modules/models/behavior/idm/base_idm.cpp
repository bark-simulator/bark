// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/models/behavior/idm/base_idm.hpp"

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
  // TODO(@hart): rename BehaviorIDMClassic
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
    25);
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

  std::pair<AgentPtr, FrenetPosition> leading_vehicle =
    observed_world.GetAgentInFront(lane_corr);
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

//! IDM Model will assume other front vehicle as constant velocity during
Trajectory BaseIDM::Plan(
    float delta_time, const world::ObservedWorld& observed_world) {
  using dynamic::StateDefinition;
  SetBehaviorStatus(BehaviorStatus::VALID);

  lane_corr_ = observed_world.GetLaneCorridor();
  if (!lane_corr_) {
    return GetLastTrajectory();
  }

  std::tuple<double, double, bool> rel_values = CalcRelativeValues(
    observed_world,
    lane_corr_);
  
  std::tuple<Trajectory, Action> traj_action =
    GenerateTrajectory(
      observed_world, lane_corr_, rel_values, delta_time);

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
