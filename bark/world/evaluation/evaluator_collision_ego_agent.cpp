// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/world/evaluation/evaluator_collision_ego_agent.hpp"

namespace bark {
namespace world {
class World;
namespace evaluation {

using bark::geometry::Point2d;
using bark::geometry::Polygon;
using bark::models::dynamic::State;
using bark::models::dynamic::StateDefinition::X_POSITION;
using bark::models::dynamic::StateDefinition::Y_POSITION;
EvaluationReturn EvaluatorCollisionEgoAgent::Evaluate(
    const world::World& world) {
  bool colliding = false;
  int num_agents = 4;
  auto ego_agent = world.GetAgent(this->agent_id_);
  if (!ego_agent) {
    return false;
  }
  State ego_state = ego_agent->GetCurrentState();
  Point2d ego_position(ego_state(X_POSITION), ego_state(Y_POSITION));
  const Polygon& ego_polygon = ego_agent->GetPolygonFromState(ego_state);
  AgentMap nearby_agents = world.GetNearestAgents(ego_position, num_agents);

  for (const auto& agent : nearby_agents) {
    if (this->agent_id_ != agent.second->GetAgentId()) {
      const Polygon& agent_polygon =
          agent.second->GetPolygonFromState(agent.second->GetCurrentState());
      if (Collide(ego_polygon, agent_polygon)) {
        colliding = true;
        break;
      }
    }
  }
  return colliding;
}

EvaluationReturn EvaluatorCollisionEgoAgent::Evaluate(
    const world::ObservedWorld& observed_world) {
  bool colliding = false;
  int num_agents = 4;
  auto ego_agent = observed_world.GetEgoAgent();
  State ego_state = ego_agent->GetCurrentState();
  Point2d ego_position(ego_state(X_POSITION), ego_state(Y_POSITION));
  const Polygon& ego_polygon = ego_agent->GetPolygonFromState(ego_state);
  AgentMap nearby_agents =
      observed_world.GetNearestAgents(ego_position, num_agents);

  for (const auto& agent : nearby_agents) {
    if (ego_agent->GetAgentId() != agent.second->GetAgentId()) {
      const Polygon& agent_polygon =
          agent.second->GetPolygonFromState(agent.second->GetCurrentState());
      if (Collide(ego_polygon, agent_polygon)) {
        colliding = true;
        break;
      }
    }
  }
  return colliding;
}

}  // namespace evaluation
}  // namespace world
}  // namespace bark
