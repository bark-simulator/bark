// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_EVALUATION_EVALUATOR_COLLISION_EGO_AGENT_HPP_
#define MODULES_WORLD_EVALUATION_EVALUATOR_COLLISION_EGO_AGENT_HPP_

#include <memory>

#include "modules/world/evaluation/base_evaluator.hpp"
#include "modules/world/world.hpp"

namespace modules {
namespace world {
namespace evaluation {

class EvaluatorCollisionEgoAgent : public BaseEvaluator {
 public:
  explicit EvaluatorCollisionEgoAgent(const AgentId& agent_id) :
    agent_id_(agent_id) { }
  virtual ~EvaluatorCollisionEgoAgent() { }
  virtual EvaluationReturn Evaluate(const world::World& world);

 private:
  AgentId agent_id_;
};

}  // namespace evaluation
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_EVALUATION_EVALUATOR_COLLISION_EGO_AGENT_HPP_
