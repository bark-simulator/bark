// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <limits>

#include "modules/models/behavior/motion_primitives/motion_primitives.hpp"
#include "modules/world/observed_world.hpp"

namespace modules {
namespace world {

using modules::geometry::Line;
using modules::geometry::Point2d;
using modules::geometry::Polygon;
using modules::models::behavior::BehaviorMotionPrimitives;
using modules::models::dynamic::State;
using modules::world::AgentMap;

std::pair<AgentPtr, Frenet> ObservedWorld::GetAgentInFrontForId(
    const AgentId& agent_id, const LaneCorridorPtr& lane_corridor) const {
  // agent_id = agent->GetAgentId();
  // State ego_state = World::GetAgents()[agent_id]->GetCurrentState();
  Point2d ego_position = World::GetAgents()[agent_id]->GetCurrentPosition();

  const Polygon& corridor_polygon = lane_corridor->GetMergedPolygon();
  const Line& center_line = lane_corridor->GetCenterLine();
  AgentMap intersecting_agents = GetAgentsIntersectingPolygon(corridor_polygon);
  if (intersecting_agents.size() == 0) {
    return std::make_pair(AgentPtr(nullptr),
                          Frenet(std::numeric_limits<double>::max(),
                                 std::numeric_limits<double>::max()));
  }

  Frenet frenet_ego(ego_position, center_line);
  double nearest_lon = std::numeric_limits<double>::max();
  double nearest_lat = std::numeric_limits<double>::max();

  AgentPtr nearest_agent(nullptr);

  for (auto it = intersecting_agents.begin(); it != intersecting_agents.end();
       ++it) {
    if (it->second->GetAgentId() == agent_id) {
      continue;
    }

    Frenet frenet_other(it->second->GetCurrentPosition(), center_line);
    double long_dist = frenet_other.lon - frenet_ego.lon;
    double lat_dist = frenet_other.lat - frenet_ego.lat;

    if (long_dist > 0.0f && long_dist < nearest_lon) {
      nearest_lon = long_dist;
      nearest_lat = lat_dist;
      nearest_agent = it->second;
    }
  }
  return std::make_pair(nearest_agent, Frenet(nearest_lon, nearest_lat));
}

std::pair<AgentPtr, Frenet> ObservedWorld::GetAgentInFront() const {
  Point2d ego_pos = CurrentEgoPosition();
  // TODO(@all): make access safe
  const auto& road_corridor = GetRoadCorridor();
  BARK_EXPECT_TRUE(road_corridor != nullptr);
  const auto& lane_corridor = road_corridor->GetCurrentLaneCorridor(ego_pos);
  BARK_EXPECT_TRUE(lane_corridor != nullptr);
  std::pair<AgentPtr, Frenet> agent_front;
  agent_front = GetAgentInFrontForId(GetEgoAgentId(), lane_corridor);

  return agent_front;
}

std::pair<AgentPtr, Frenet> ObservedWorld::GetAgentBehind() const {
  // TODO(@Klemens): implement
  double nearest_lon = std::numeric_limits<double>::max();
  double nearest_lat = std::numeric_limits<double>::max();

  AgentPtr nearest_agent(nullptr);

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
          next_world->GetEgoBehaviorModel());
  if (ego_behavior_model) {
    ego_behavior_model->ActionToBehavior(ego_action);
  } else {
    LOG(ERROR) << "Currently only BehaviorMotionPrimitive model supported for "
                  "ego prediction, adjust prediction settings.";  // NOLINT
  }
  next_world->Step(time_span);
  return std::dynamic_pointer_cast<World>(next_world);
}

}  // namespace world
}  // namespace modules
