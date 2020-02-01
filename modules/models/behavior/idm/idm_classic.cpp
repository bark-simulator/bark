// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include "modules/models/behavior/idm/idm_classic.hpp"
#include "modules/world/observed_world.hpp"
#include "modules/world/map/frenet.hpp"

#include <math.h>

namespace modules {
namespace models {
namespace behavior {

using modules::models::dynamic::State;
using modules::models::dynamic::StateDefinition;
using modules::world::objects::AgentPtr;
using modules::world::objects::Agent;
using modules::geometry::Point2d;
using modules::world::map::FrenetPosition;

double BehaviorIDMClassic::CalculateLongitudinalAccelerationTwoAgents(const std::shared_ptr<const Agent>& ego_agent, const std::shared_ptr<const Agent>& leading_agent, const double distance) {

  // Parameters
  const float desired_velocity = get_desired_velocity();
  const float minimum_spacing = get_minimum_spacing();
  const float desired_time_headway = get_desired_time_headway();
  const float max_acceleration = get_max_acceleration();
  const float comfortable_braking_acceleration = get_comfortable_braking_acceleration();
  const int exponent = get_exponent();

  // relative velocity and longitudinal distance
  const State ego_state = ego_agent->GetCurrentState();
  const float ego_velocity = ego_state(StateDefinition::VEL_POSITION);
  double interaction_term = 0.0f;
  if(leading_agent) {
    // Leading vehicle exists in driving corridor, we calculate interaction term

    const State lead_state = leading_agent->GetCurrentState();

    const float vehicle_length = ego_agent->GetShape().front_dist_ +
                                  leading_agent->GetShape().rear_dist_;
    // TODO: that's not correct!
    const double net_distance = distance - vehicle_length - 0.0f ; // For now assume ego longitudinal state at start of driving corridor
    const double net_velocity = ego_state(StateDefinition::VEL_POSITION) - lead_state(StateDefinition::VEL_POSITION);

    const double helper_state = minimum_spacing + ego_velocity*desired_time_headway +
                      (ego_velocity*net_velocity) / (2*sqrt(max_acceleration*comfortable_braking_acceleration));
    
    BARK_EXPECT_TRUE(!std::isnan(helper_state));
    interaction_term = (helper_state / net_distance)*(helper_state / net_distance);
    BARK_EXPECT_TRUE(!std::isnan(interaction_term));
  }
  
  // Calculate acceleration based on https://en.wikipedia.org/wiki/Intelligent_driver_model (Maybe look into paper for other implementation aspects)  
  return max_acceleration * ( 1 - pow(ego_velocity / desired_velocity, exponent) - interaction_term);
}

double BehaviorIDMClassic::CalculateLongitudinalAcceleration(const ObservedWorld& observed_world) {

  std::pair<AgentPtr, modules::world::map::FrenetPosition> leading_vehicle = observed_world.GetAgentInFront();
  std::shared_ptr<const Agent> ego_agent = observed_world.GetEgoAgent();

  modules::world::map::FrenetPosition lead_veh_frenet = leading_vehicle.second;
  double distance = lead_veh_frenet.lon;

  // TODO(@Klemens): Distance refers to braking point, should be renamed / clarified
  double acc = CalculateLongitudinalAccelerationTwoAgents(ego_agent, leading_vehicle.first, distance);
  return acc;
}

}  // namespace behavior
}  // namespace models
}  // namespace modules
