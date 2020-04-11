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



BehaviorIDMClassic::BehaviorIDMClassic(const commons::ParamsPtr& params) : BehaviorModel(params)
             {
    
    param_minimum_spacing_ = params->GetReal("BehaviorIDMClassic::MinimumSpacing", "See Wikipedia IDM article", 2.0f);
    param_desired_time_head_way_ = params->GetReal("BehaviorIDMClassic::DesiredTimeHeadway", "See Wikipedia IDM article", 1.5f);
    param_max_acceleration_ = params->GetReal("BehaviorIDMClassic::MaxAcceleration", "See Wikipedia IDM article", 1.7f);
    param_acceleration_lower_bound_ = params->GetReal("BehaviorIDMClassic::AccelerationLowerBound", "See Wikipedia IDM article", -5.0f);
    param_acceleration_upper_bound_ = params->GetReal("BehaviorIDMClassic::AccelerationUpperBound", "See Wikipedia IDM article", 8.0f);
    param_desired_velocity_ = params->GetReal("BehaviorIDMClassic::DesiredVelocity", "See Wikipedia IDM article", 15.0f);
    param_comfortable_braking_acceleration_ = params->GetReal("BehaviorIDMClassic::ComfortableBrakingAcceleration", "See Wikipedia IDM article", 1.67f);
    param_min_velocity_ = params->GetReal("BehaviorIDMClassic::MinVelocity", "See Wikipedia IDM article", 0.0f);
    param_max_velocity_ = params->GetReal("BehaviorIDMClassic::MaxVelocity", "See Wikipedia IDM article", 50.0f);
    param_coolness_factor_ = params->GetReal("BehaviorIDMClassic::CoolnessFactor", "If non zero, constant accleration heuristic is applied", 0.0f);
    SetLastAction(Continuous1DAction(0.0f));
}


double BehaviorIDMClassic::CalcFreeRoadTerm(const double vel_ego) const {
  const float desired_velocity = GetDesiredVelocity();
  const int exponent = GetExponent();

  double free_road_term = 1 - pow(vel_ego / desired_velocity, exponent);
  return free_road_term;
}

double BehaviorIDMClassic::CalcInteractionTerm(double net_distance,
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

double BehaviorIDMClassic::CalcNetDistance(
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

double BehaviorIDMClassic::CalcIDMAcc(const double net_distance,
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
double BehaviorIDMClassic::CalcRawIDMAcc(const double& net_distance,
                                         const double& vel_ego,
                                         const double& vel_other) const {
  const double free_road_term = CalcFreeRoadTerm(vel_ego);
  const double interaction_term =
      CalcInteractionTerm(net_distance, vel_ego, vel_other);
  return GetMaxAcceleration() * (free_road_term - interaction_term);
}

double CalcCAHAcc(const double& net_distance, const double& vel_ego,
                       const double& vel_other, const double& acc_ego,
                       const double& acc_other) {
  // implements equation 11.25 on on page 198
  
  const float max_acceleration = GetMaxAcceleration();
  const double effect_acc_other = std::min(acc_other, max_acceleration);
  if(vel_other*(vel_ego - vel_other) <= -2*net_distance*effect_acc_other) {
    return vel_ego*vel_ego*effect_acc_other / (vel_other*vel_other - 2*net_distance*effect_acc_other);
  } else {
    const double step_function = vel_ego - vel_other >= 0 ? 1.0 : 0.0 
    return effect_acc_other - (vel_ego - vel_other)*(vel_ego - vel_other) * step_function / (2* net_distance);
  }
}

double CalcACCAcc(const double& net_distance, const double& vel_ego,
                       const double& vel_other, const double& acc_ego,
                       const double& acc_other) const {
  // implements equation 11.26 on on page 199
  const double idm_acc = CalcRawIDMAcc(net_distance, vel_ego, vel_other);
  if (param_coolness_factor_ = 0.0f) {
    return idm_acc;
  }

  const double cah_acc = CalcCAHAcc(net_distance, vel_ego, vel_other,
                                   acc_ego, acc_other);
  const double b = GetComfortableBrakingAcceleration();
  double acc = 0.0f;
  if (idm_acc >= cah_acc) {
    acc = idm_acc;
  } else {
    acc = (1 - param_coolness_factor_) * idm_acc + param_coolness_factor_ * ( cah_acc + b * tanh( (idm_acc - cah_acc) / b ) );
  }

  const float acc_lower_bound = GetAccelerationLowerBound();
  const float acc_upper_bound = GetAccelerationUpperBound();
  acc = std::max(std::min(acc, acc_upper_bound), acc_lower_bound);
  return acc
}

//! IDM Model will assume other front vehicle as constant velocity during
//! delta_time
Trajectory BehaviorIDMClassic::Plan(
    float delta_time, const world::ObservedWorld& observed_world) {
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

  double net_distance = std::numeric_limits<double>::max();
  double vel_other = 0.0f;
  double acc_other = 0.0f;
  if (leading_vehicle.first) {
    net_distance = CalcNetDistance(ego_agent, leading_vehicle.first);
    dynamic::State other_vehicle_state =
        leading_vehicle.first->GetCurrentState();
    vel_other = other_vehicle_state(StateDefinition::VEL_POSITION);

    // Get acceleration action other
    if (param_coolness_factor_ > 0.0f) {
      auto last_action = leading_vehicle.first->GetBehaviorModel()->GetLastAction();
      if(last_action.type() == typeid(Continuous1DAction)) {
        acc_other = boost::get<Continuous1DAction>(last_action);
      } else if (action.type() == typeid(LonLatAction)) {
        acc_other = boost::get<LonLatAction>(last_action).acc_lon;
      } else {
        LOG(WARNING) << "Other's action not known for cah calculation";
      }
    }
  }

  if (!line.obj_.empty()) {
    // adding state at t=0
    traj.block<1, StateDefinition::MIN_STATE_SIZE>(0, 0) =
        ego_vehicle_state.transpose().block<1, StateDefinition::MIN_STATE_SIZE>(
            0, 0);

    float s_start = GetNearestS(line, pose);  // checked
    double start_time = observed_world.GetWorldTime();
    float vel_i = ego_vehicle_state(StateDefinition::VEL_POSITION);
    float acc_i = boost::get<Continuous1DAction>(GetLastAction());
    float s_i = s_start;
    double t_i;
    double acc;
    double traveled_ego;
    double traveled_other;
    double initial_acceleration;

    for (int i = 1; i < num_traj_time_points; ++i) {
      acc = CalcACCAcc(net_distance, vel_i, vel_other, acc_i, acc_other);
      traveled_ego = +0.5f * acc * dt * dt + vel_i * dt;
      traveled_other = vel_other * dt;
      net_distance += traveled_other - traveled_ego;

      // Set initial acceleration to maintain action value
      if(i == 1) {
        initial_acceleration = acc;
      }

      BARK_EXPECT_TRUE(!std::isnan(acc));

      s_i += +0.5f * acc * dt * dt + vel_i * dt;
      const float temp_velocity = vel_i + acc * dt;
      vel_i = std::max(std::min(temp_velocity, max_velocity), min_velocity);
      t_i = static_cast<float>(i) * dt + start_time;

      geometry::Point2d traj_point = GetPointAtS(line, s_i);  // checked
      float traj_angle = GetTangentAngleAtS(line, s_i);       // checked

      BARK_EXPECT_TRUE(!std::isnan(boost::geometry::get<0>(traj_point)));
      BARK_EXPECT_TRUE(!std::isnan(boost::geometry::get<1>(traj_point)));
      BARK_EXPECT_TRUE(!std::isnan(traj_angle));

      traj(i, StateDefinition::TIME_POSITION) = t_i;  // checked
      traj(i, StateDefinition::X_POSITION) =
          boost::geometry::get<0>(traj_point);  // checked
      traj(i, StateDefinition::Y_POSITION) =
          boost::geometry::get<1>(traj_point);                // checked
      traj(i, StateDefinition::THETA_POSITION) = traj_angle;  // checked
      traj(i, StateDefinition::VEL_POSITION) = vel_i;         // checked
    }

    SetLastAction(Action(Continuous1DAction(initial_acceleration)));
  }
  this->SetLastTrajectory(traj);
  return traj;
}

}  // namespace behavior
}  // namespace models
}  // namespace modules
