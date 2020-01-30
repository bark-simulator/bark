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

//! TODO(@Klemens): move to world
FrontRearAgents ObservedWorld::GetAgentFrontRearForId(
    const AgentId& agent_id, const LaneCorridorPtr& lane_corridor) const {
  FrontRearAgents fr_agents;
  Point2d ego_position = World::GetAgents()[agent_id]->GetCurrentPosition();

  const Polygon& corridor_polygon = lane_corridor->GetMergedPolygon();
  const Line& center_line = lane_corridor->GetCenterLine();
  AgentMap intersecting_agents = GetAgentsIntersectingPolygon(corridor_polygon);
  if (intersecting_agents.size() == 0) {
    fr_agents.front = std::make_pair(AgentPtr(nullptr), Frenet(0, 0));
    fr_agents.rear = fr_agents.front;
    return fr_agents;
  }

  Frenet frenet_ego(ego_position, center_line);
  const double numeric_max = std::numeric_limits<double>::max();

  double nearest_lon_front = numeric_max, nearest_lat_front = numeric_max,
         nearest_lon_rear = numeric_max, nearest_lat_rear = numeric_max;

  AgentPtr nearest_agent_front(nullptr);
  AgentPtr nearest_agent_rear(nullptr);

  for (auto it = intersecting_agents.begin(); it != intersecting_agents.end();
       ++it) {
    if (it->second->GetAgentId() == agent_id) {
      continue;
    }

    Frenet frenet_other(it->second->GetCurrentPosition(), center_line);
    double long_dist = frenet_other.lon - frenet_ego.lon;
    double lat_dist = frenet_other.lat - frenet_ego.lat;

    if (long_dist > 0.0f && long_dist < nearest_lon_front) {
      nearest_lon_front = long_dist;
      nearest_lat_front = lat_dist;
      nearest_agent_front = it->second;
    } else if (long_dist < 0.0f &&
               std::abs(long_dist) < std::abs(nearest_lon_rear)) {
      nearest_lon_rear = long_dist;
      nearest_lat_rear = lat_dist;
      nearest_agent_rear = it->second;
    }
  }

  Frenet frenet_front = Frenet(nearest_lon_front, nearest_lat_front);
  fr_agents.front = std::make_pair(nearest_agent_front, frenet_front);
  Frenet frenet_rear = Frenet(nearest_lon_rear, nearest_lat_rear);
  fr_agents.rear = std::make_pair(nearest_agent_rear, frenet_rear);

  return fr_agents;
}

FrontRearAgents ObservedWorld::GetAgentFrontRear() const {
  Point2d ego_pos = CurrentEgoPosition();
  const auto& road_corridor = GetRoadCorridor();
  BARK_EXPECT_TRUE(road_corridor != nullptr);
  const auto& lane_corridor = road_corridor->GetCurrentLaneCorridor(ego_pos);
  BARK_EXPECT_TRUE(lane_corridor != nullptr);

  AgentId id = GetEgoAgentId();
  FrontRearAgents fr_agent = GetAgentFrontRearForId(id, lane_corridor);

  return fr_agent;
}

AgentFrenetPair ObservedWorld::GetAgentInFront() const {
  FrontRearAgents fr_agent = GetAgentFrontRear();
  return fr_agent.front;
}

AgentFrenetPair ObservedWorld::GetAgentBehind() const {
  FrontRearAgents fr_agent = GetAgentFrontRear();
  return fr_agent.rear;
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
