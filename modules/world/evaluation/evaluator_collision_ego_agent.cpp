// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/world/evaluation/evaluator_collision_ego_agent.hpp"

namespace modules {
namespace world {
class World;
namespace evaluation {

using modules::models::dynamic::State;
using modules::models::dynamic::StateDefinition::X_POSITION;
using modules::models::dynamic::StateDefinition::Y_POSITION;
using modules::geometry::Polygon;
using modules::geometry::Point2d;
EvaluationReturn EvaluatorCollisionEgoAgent::Evaluate(
  const world::World &world) {
  bool colliding = false;
  int num_agents = 4;
  AgentPtr ego_agent = world.get_agent(this->agent_id_);
  State ego_state = ego_agent->get_current_state();
  Point2d ego_position(ego_state(X_POSITION),
                       ego_state(Y_POSITION));
  Polygon ego_polygon =
    ego_agent->GetPolygonFromState(ego_state);
  AgentMap nearby_agents = world.GetNearestAgents(ego_position, num_agents);

  for (auto agent : nearby_agents) {
    if (ego_agent->get_agent_id() != agent.second->get_agent_id()) {
      Polygon agent_polygon =
        agent.second->GetPolygonFromState(agent.second->get_current_state());
      if ( Collide(ego_polygon, agent_polygon) ) {
        colliding = true;
        break;
      }
    }
  }
  return colliding;
}

}  // namespace evaluation
}  // namespace world
}  // namespace modules
