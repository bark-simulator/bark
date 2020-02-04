// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include "modules/models/behavior/idm/idm_classic.hpp"
#include "modules/world/observed_world.hpp"
#include "modules/commons/transformation/frenet.hpp"

#include <math.h>

namespace modules {
namespace models {
namespace behavior {

using modules::models::dynamic::State;
using modules::models::dynamic::StateDefinition;
using modules::world::objects::AgentPtr;
using modules::world::objects::Agent;
using modules::geometry::Point2d;
using modules::commons::transformation::FrenetPosition;

double BehaviorIDMClassic::CalcInteractionTerm(const double net_distance, const double vel_ego, const double vel_other) const {

  // Parameters
  const float minimum_spacing = get_minimum_spacing();
  const float desired_time_headway = get_desired_time_headway();
  const float max_acceleration = get_max_acceleration();
  const float comfortable_braking_acceleration = get_comfortable_braking_acceleration();

  const double net_velocity = vel_ego - vel_other;

  const double helper_state = minimum_spacing + vel_ego*desired_time_headway +
                      (vel_ego*net_velocity) / (2*sqrt(max_acceleration*comfortable_braking_acceleration));
    
  BARK_EXPECT_TRUE(!std::isnan(helper_state));
  double interaction_term = (helper_state / net_distance)*(helper_state / net_distance);
  BARK_EXPECT_TRUE(!std::isnan(interaction_term));

  return interaction_term;

}

double BehaviorIDMClassic::CalcIDMAcc(const std::shared_ptr<const Agent>& ego_agent, const std::shared_ptr<const Agent>& leading_agent) {

  const float desired_velocity = get_desired_velocity();
  const int exponent = get_exponent();
  const float max_acceleration = get_max_acceleration();

  // relative velocity and longitudinal distance
  const State ego_state = ego_agent->GetCurrentState();
  const auto& ego_lane_corridor = ego_agent->GetRoadCorridor()->GetCurrentLaneCorridor(ego_agent->GetCurrentPosition());
  FrenetPosition frenet_ego(ego_agent->GetCurrentPosition(), ego_lane_corridor->GetCenterLine());

  const float ego_velocity = ego_state(StateDefinition::VEL_POSITION);

  double interaction_term = 0;

  if(leading_agent) {
    // Leading vehicle exists in driving corridor, we calculate interaction term
    const State leading_state = leading_agent->GetCurrentState();
    const float other_velocity = leading_state(StateDefinition::VEL_POSITION);

    const auto& leading_lane_corridor = leading_agent->GetRoadCorridor()->GetCurrentLaneCorridor(leading_agent->GetCurrentPosition());
    FrenetPosition frenet_leading(leading_agent->GetCurrentPosition(), leading_lane_corridor->GetCenterLine());

    const float vehicle_length = ego_agent->GetShape().front_dist_ +
                                  leading_agent->GetShape().rear_dist_;
    const double net_distance = frenet_leading.lon - vehicle_length - frenet_ego.lon;

    interaction_term = CalcInteractionTerm(net_distance, ego_velocity, other_velocity);
  }
  
  // Calculate acceleration based on https://en.wikipedia.org/wiki/Intelligent_driver_model
  return max_acceleration * ( 1 - pow(ego_velocity / desired_velocity, exponent) - interaction_term);
}

double BehaviorIDMClassic::CalculateLongitudinalAcceleration(const ObservedWorld& observed_world) {

  std::pair<AgentPtr, FrenetPosition> leading_vehicle = observed_world.GetAgentInFront();
  std::shared_ptr<const Agent> ego_agent = observed_world.GetEgoAgent();


  // TODO(@Klemens): Distance refers to braking point, should be renamed / clarified
  double acc = CalcIDMAcc(ego_agent, leading_vehicle.first);
  return acc;
}

}  // namespace behavior
}  // namespace models
}  // namespace modules
