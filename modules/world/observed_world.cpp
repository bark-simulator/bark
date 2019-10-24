// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <limits>

#include "modules/world/observed_world.hpp"
#include "modules/models/behavior/motion_primitives/motion_primitives.hpp"

namespace modules {
namespace world {

using modules::geometry::Polygon;
using modules::world::AgentMap;
using modules::world::map::Frenet;
using modules::models::behavior::BehaviorMotionPrimitives;

std::pair<AgentPtr, Frenet> ObservedWorld::get_agent_in_front() const {
  State ego_state = current_ego_state();

  const auto& driving_corridor = get_local_map()->get_horizon_driving_corridor();
  const Polygon& corridor_polygon = driving_corridor.CorridorPolygon(); 
  AgentMap intersecting_agents =  GetAgentsIntersectingPolygon(corridor_polygon);
  if(intersecting_agents.size() == 0) {
    return std::make_pair(AgentPtr(nullptr),
                          Frenet(std::numeric_limits<double>::max(),
                          std::numeric_limits<double>::max()));
  }


  Frenet frenet_ego = driving_corridor.FrenetFromCenterLine(current_ego_position());
  double nearest_lon = std::numeric_limits<double>::max();
  double nearest_lat = std::numeric_limits<double>::max();

  AgentPtr nearest_agent(nullptr);

  for (auto aiter = intersecting_agents.begin(); aiter != intersecting_agents.end(); ++aiter) {
    if (aiter->second->get_agent_id() == ego_agent_id_) {
      continue;
    }

    Frenet frenet_other = driving_corridor.FrenetFromCenterLine(
      aiter->second->get_current_position());
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

void ObservedWorld::SetupPrediction(const PredictionSettings& settings) {
    settings.ApplySettings(*this);
}

WorldPtr ObservedWorld::Predict(float time_span) const {
  std::shared_ptr<ObservedWorld> next_world =
    std::dynamic_pointer_cast<ObservedWorld>(ObservedWorld::Clone());
  next_world->Step(time_span);
  return std::dynamic_pointer_cast<World>(next_world);
}

WorldPtr ObservedWorld::Predict(float time_span,
                                const DiscreteAction& ego_action) const {
  std::shared_ptr<ObservedWorld> next_world =
    std::dynamic_pointer_cast<ObservedWorld>(ObservedWorld::Clone());
  std::shared_ptr<BehaviorMotionPrimitives> ego_behavior_model =
     std::dynamic_pointer_cast<BehaviorMotionPrimitives>(
       next_world->get_ego_behavior_model());
  if (ego_behavior_model) {
      ego_behavior_model->ActionToBehavior(ego_action);
  } else {
    LOG(ERROR) << "Currently only BehaviorMotionPrimitive model supported for ego prediction, adjust prediction settings.";  // NOLINT
  }
  next_world->Step(time_span);
  return std::dynamic_pointer_cast<World>(next_world);
}

}  // namespace world
}  // namespace modules
