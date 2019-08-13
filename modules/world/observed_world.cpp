// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include "modules/world/observed_world.hpp"
#include <limits>

namespace modules {
namespace world {

using modules::geometry::Polygon;
using modules::world::AgentMap;
using modules::world::map::Frenet;

std::pair<AgentPtr, modules::world::map::Frenet> ObservedWorld::get_agent_in_front() const {
  State ego_state = current_ego_state();

  const auto& driving_corridor = get_local_map()->get_horizon_driving_corridor();
  const Polygon& corridor_polygon = driving_corridor.CorridorPolygon(); 
  AgentMap intersecting_agents =  GetAgentsIntersectingPolygon(corridor_polygon);
  if(intersecting_agents.size() == 0) {
    return std::make_pair(AgentPtr(nullptr), Frenet(std::numeric_limits<double>::max(),
                                              std::numeric_limits<double>::max()));
  }

  
  Frenet frenet_ego = driving_corridor.FrenetFromCenterLine(current_ego_position());
  double nearest_lon = std::numeric_limits<double>::max();
  double nearest_lat = std::numeric_limits<double>::max();

  AgentPtr nearest_agent(nullptr);
  
  for (auto aiter = intersecting_agents.begin(); aiter != intersecting_agents.end(); ++aiter) {
    if(aiter->second->get_agent_id() == ego_agent_id_) {
      continue;
    }

    Frenet frenet_other = driving_corridor.FrenetFromCenterLine(aiter->second->get_current_position());
    double long_dist = frenet_other.lon - frenet_ego.lon;
    double lat_dist = frenet_other.lat - frenet_ego.lat;

    if (long_dist > 0.0f && long_dist < nearest_lon) {
      nearest_lon = long_dist;
      nearest_lat = lat_dist;
      nearest_agent = aiter->second;
    }
  }
  return std::make_pair(nearest_agent, Frenet(nearest_lon, nearest_lat));
}

void ObservedWorld::Step(const float time_step) {
  World::Step(time_step);
}

ObservedWorld *ObservedWorld::Clone() const {
  ObservedWorld *new_observed_world = new ObservedWorld(*this);
  new_observed_world->clear_all();
  for (auto const &agent : get_agents()) {
    new_observed_world->add_agent(AgentPtr(agent.second->Clone()));
  }
  for (auto const &object : get_objects()) {
    new_observed_world->add_object(ObjectPtr(object.second->Clone()));
  }
  return new_observed_world;
}

}  // namespace world
}  // namespace modules
