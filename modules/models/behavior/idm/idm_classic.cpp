// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include "modules/models/behavior/idm/idm_classic.hpp"
#include "modules/world/observed_world.hpp"

#include <math.h>

namespace modules {
namespace models {
namespace behavior {

using modules::models::dynamic::State;
using modules::models::dynamic::StateDefinition;
using modules::world::objects::AgentPtr;
using modules::geometry::Point2d;

double BehaviorIDMClassic::CalculateLongitudinalAcceleration(const ObservedWorld& observed_world) {


  // Parameters
  const float desired_velocity = get_desired_velocity();
  const float minimum_spacing = get_minimum_spacing();
  const float desired_time_headway = get_desired_time_headway();
  const float max_acceleration = get_max_acceleration();
  const float comfortable_braking_acceleration = get_comfortable_braking_acceleration();
  const int exponent = get_exponent();


  // relative velocity and longitudinal distance
  const State ego_state = observed_world.current_ego_state();
  const float ego_velocity = ego_state(StateDefinition::VEL_POSITION);
  auto leading_vehicle = GetLeadingVehicle(observed_world);
  double interaction_term = 0.0f;
  if(leading_vehicle.first) {
    // Leading vehicle exists in driving corridor, we calculate interaction term

    const State lead_state = leading_vehicle.first->get_current_state();
    modules::world::map::Frenet lead_veh_frenet = leading_vehicle.second;

    const float vehicle_length = observed_world.get_ego_agent()->get_shape().front_dist_ +
                                  leading_vehicle.first->get_shape().rear_dist_;
    const double net_distance = lead_veh_frenet.lon - vehicle_length - 0.0f ; // For now assume ego longitudinal state at start of driving corridor
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

std::pair<AgentPtr, modules::world::map::Frenet> BehaviorIDMClassic::GetLeadingVehicle(const ObservedWorld& observed_world) {
  std::pair<AgentPtr, modules::world::map::Frenet> leading_vehicle = observed_world.get_agent_in_front();
  return leading_vehicle;
}

}  // namespace behavior
}  // namespace models
}  // namespace modules
