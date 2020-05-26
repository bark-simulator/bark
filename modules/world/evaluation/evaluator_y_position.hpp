// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_EVALUATION_EVALUATOR_Y_POSITION_HPP_
#define MODULES_WORLD_EVALUATION_EVALUATOR_Y_POSITION_HPP_

#include <limits>
#include <memory>

#include "modules/world/evaluation/base_evaluator.hpp"
#include "modules/world/observed_world.hpp"
#include "modules/world/world.hpp"

namespace modules {
namespace world {
namespace evaluation {
using modules::geometry::Point2d;
using modules::geometry::Polygon;
using modules::models::dynamic::State;
using modules::models::dynamic::StateDefinition::Y_POSITION;

class EvaluatorYPosition : public BaseEvaluator {
 public:
  EvaluatorYPosition() : agent_id_(std::numeric_limits<AgentId>::max()) {}
  explicit EvaluatorYPosition(const AgentId& agent_id) : agent_id_(agent_id) {}
  virtual ~EvaluatorYPosition() {}
  virtual EvaluationReturn Evaluate(const world::World& world) {
    const auto& agent = world.GetAgent(agent_id_);
    BARK_EXPECT_TRUE(bool(agent));
    if (bool(agent)){
      return EvaluatorYPosition::GetY(agent);
    }
    return -1;
  }

  static float GetY(
    const std::shared_ptr<const modules::world::objects::Agent>& agent) {
    const auto& agent_pos = agent->GetCurrentPosition();
    return agent_pos.get<1>();
  }

 private:
  AgentId agent_id_;
};

}  // namespace evaluation
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_EVALUATION_EVALUATOR_Y_POSITION_HPP_
