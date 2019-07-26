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

double BehaviorIDMClassic::CalculateLongitudinalAcceleration(const ObservedWorld& observed_world) {
  using modules::models::dynamic::State;
  using modules::models::dynamic::StateDefinition;
  using modules::world::objects::AgentPtr;
  using modules::geometry::Point2d;

  // Parameters
  const float desired_velocity = 50;
  const float minimum_spacing = 1;
  const float desired_time_headway = 4;
  const float max_acceleration = 5;
  const float comfortable_braking_acceleration = 4;
  const int exponent = 4;


  // Calculate relative velocity and longitudinal distance
  const State ego_state = observed_world.get_ego_state();
  const float ego_velocity = ego_state(StateDefinition::VEL_POSITION);

  AgentPtr lead_veh = GetLeadingVehicle(observed_world);
  const State lead_state = lead_veh->get_current_state();
  modules::world::map::Frenet lead_veh_frenet =
       observed_world.get_local_map()->get_horizon_driving_corridor().FrenetFromCenterLine(Point2d(
              lead_state(StateDefinition::X_POSITION), lead_state(StateDefinition::Y_POSITION)));

  const float vehicle_length = observed_world.get_ego_agent()->get_shape().front_dist_ +
                                lead_veh->get_shape().rear_dist_;
  const float net_distance = lead_veh_frenet.lon - vehicle_length - 0.0f ; // For now assume ego longitudinal state at start of driving corridor
  const float net_velocity = ego_state(StateDefinition::VEL_POSITION) - lead_state(StateDefinition::VEL_POSITION);


  // Calculate acceleration based on https://en.wikipedia.org/wiki/Intelligent_driver_model (Maybe look into paper for other implementation aspects)
  const float helper_state = minimum_spacing + ego_velocity*desired_time_headway +
                     (ego_velocity*net_velocity) / (2*sqrt(max_acceleration*comfortable_braking_acceleration));

  return max_acceleration * ( 1 - pow(ego_velocity / desired_velocity, exponent) - pow(helper_state / net_distance, 2));
}

modules::world::objects::AgentPtr BehaviorIDMClassic::GetLeadingVehicle(const ObservedWorld& observed_world) {
  modules::world::AgentMap front_vehicles = observed_world.get_agents_in_front(1);
  assert(front_vehicles.size()==1);
  return front_vehicles.begin()->second;
}

}  // namespace behavior
}  // namespace models
}  // namespace modules
