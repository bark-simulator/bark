// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/world/evaluation/evaluator_collision_driving_corridor.hpp"
#include "modules/world/world.hpp"

namespace modules {
namespace world {
namespace evaluation {

EvaluationReturn EvaluatorCollisionDrivingCorridor::Evaluate(const world::World &world) {
  // checks collision with inner and outer line of driving corridor
  // assumption: agent is initially inside the local map and the world steps are fine
  //   to prevent the agent from "jumping" outside the driving corridor

  modules::geometry::Polygon poly_agent;
  bool colliding = false;
  modules::geometry::Line lane_inner;
  modules::geometry::Line lane_outer;

  for (auto agent : world.get_agents()) {
    poly_agent = agent.second->GetPolygonFromState(
      agent.second->get_current_state());
    const auto corridor_poly = agent.second->get_local_map()->get_driving_corridor().CorridorPolygon();
    
    if (!Within(poly_agent, corridor_poly)) {
      colliding = true;
      break;
    }
  }
  return colliding;
}

}  // namespace evaluation
}  // namespace world
}  // namespace modules