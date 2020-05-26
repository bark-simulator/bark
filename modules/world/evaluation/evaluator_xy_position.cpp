// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/world/evaluation/evaluator_xy_position.hpp"

namespace modules {
namespace world {
class World;
namespace evaluation {

using modules::geometry::Point2d;
using modules::geometry::Polygon;
using modules::models::dynamic::State;
using modules::models::dynamic::StateDefinition::X_POSITION;
using modules::models::dynamic::StateDefinition::Y_POSITION;

EvaluationReturn EvaluatorXyPosition::Evaluate(const world::World& world) {
  const auto& agent = world.GetAgent(agent_id_);
  State state = agent->GetCurrentState();

  if (y_) {
    return state(Y_POSITION);
  } else {
    return state(X_POSITION);
  }
}

}  // namespace evaluation
}  // namespace world
}  // namespace modules
