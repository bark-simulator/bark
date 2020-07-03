// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/world/evaluation/evaluator_collision_agents.hpp"
#include "bark/world/world.hpp"

namespace bark {
namespace world {
namespace evaluation {

EvaluationReturn EvaluatorCollisionAgents::Evaluate(const world::World& world) {
  bark::geometry::Polygon poly_agent1;
  bark::geometry::Polygon poly_agent2;
  bool colliding = false;

  for (auto agent_outer : world.GetValidAgents()) {
    poly_agent1 = agent_outer.second->GetPolygonFromState(
        agent_outer.second->GetCurrentState());
    for (auto agent_inner : world.GetValidAgents()) {
      poly_agent2 = agent_inner.second->GetPolygonFromState(
          agent_inner.second->GetCurrentState());
      if (agent_inner.first != agent_outer.first) {
        if (Collide(poly_agent1, poly_agent2)) {
          colliding = true;
          break;
        }
      }
      if (colliding == true) break;
    }
  }
  return colliding;
}

}  // namespace evaluation
}  // namespace world
}  // namespace bark
