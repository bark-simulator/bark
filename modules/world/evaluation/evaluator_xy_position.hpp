// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_EVALUATION_EVALUATOR_XY_POSITION_HPP_
#define MODULES_WORLD_EVALUATION_EVALUATOR_XY_POSITION_HPP_

#include <limits>
#include <memory>

#include "modules/world/evaluation/base_evaluator.hpp"
#include "modules/world/observed_world.hpp"
#include "modules/world/world.hpp"

namespace modules {
namespace world {
namespace evaluation {

class EvaluatorXyPosition : public BaseEvaluator {
 public:
  EvaluatorXyPosition()
      : agent_id_(std::numeric_limits<AgentId>::max()), y_(false) {}
  explicit EvaluatorXyPosition(const AgentId& agent_id, const bool y) : agent_id_(agent_id), y_(y) {}
  virtual ~EvaluatorXyPosition() {}
  virtual EvaluationReturn Evaluate(const world::World& world);

 private:
  AgentId agent_id_;
  bool y_;
};

}  // namespace evaluation
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_EVALUATION_EVALUATOR_XY_POSITION_HPP_
