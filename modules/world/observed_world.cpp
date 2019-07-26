// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include "modules/world/observed_world.hpp"

namespace modules {
namespace world {

using modules::geometry::Polygon;
using modules::world::AgentMap;

AgentMap ObservedWorld::get_agents_in_front(const unsigned int& max_num) const {
  State ego_state = get_ego_state();
  AgentMap nearest_agents = GetNearestAgents(get_ego_point(), max_num);

  const Polygon& corridor_polygon = get_local_map()->get_horizon_driving_corridor().CorridorPolygon(); 
  AgentMap front_agents;
  for (auto& agent : nearest_agents) {
    if(modules::geometry::Collide(corridor_polygon, agent.second->GetPolygonFromState(agent.second->get_current_state()))) {
      front_agents[agent.first] = agent.second;
    }
  }
  return front_agents;
}

}  // namespace world
}  // namespace modules