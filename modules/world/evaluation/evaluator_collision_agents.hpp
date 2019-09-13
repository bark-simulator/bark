// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_EVALUATION_COLLISION_AGENTS_HPP_
#define MODULES_WORLD_EVALUATION_COLLISION_AGENTS_HPP_

#include "modules/world/evaluation/base_evaluator.hpp"

namespace modules {
namespace world {
class World;
namespace evaluation {

class EvaluatorCollisionAgents : public BaseEvaluator {
 public:
  EvaluatorCollisionAgents()  {}
  virtual ~EvaluatorCollisionAgents() {}
  virtual EvaluationReturn Evaluate(const world::World& world);
};

}  // namespace evaluation
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_EVALUATION_COLLISION_AGENTS_HPP_
