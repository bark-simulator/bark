// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/world/evaluation/evaluator_collision_agents.hpp"
#include "modules/world/world.hpp"

namespace modules {
namespace world {
namespace evaluation {

EvaluationReturn EvaluatorCollisionAgents::Evaluate(const world::World& world) {
  modules::geometry::Polygon poly_agent1;
  modules::geometry::Polygon poly_agent2;
  bool colliding = false;

  for (auto agent_outer : world.get_agents()) {
    poly_agent1 = agent_outer.second->GetPolygonFromState(
      agent_outer.second->get_current_state());
    for (auto agent_inner : world.get_agents()) {
      poly_agent2 = agent_inner.second->GetPolygonFromState(
        agent_inner.second->get_current_state());
      if (agent_inner.first != agent_outer.first) {
        if (Collide(poly_agent1, poly_agent2)) {
          colliding = true;
          break;
        }
      }
      if (colliding == true)
        break;
    }
  }
  return colliding;
}

}  // namespace evaluation
}  // namespace world
}  // namespace modules
