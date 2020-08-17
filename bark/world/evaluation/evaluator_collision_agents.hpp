// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_EVALUATION_COLLISION_AGENTS_HPP_
#define BARK_WORLD_EVALUATION_COLLISION_AGENTS_HPP_

#include "bark/world/evaluation/base_evaluator.hpp"

namespace bark {
namespace world {
class World;
namespace evaluation {

class EvaluatorCollisionAgents : public BaseEvaluator {
 public:
  EvaluatorCollisionAgents() {}
  virtual ~EvaluatorCollisionAgents() {}
  virtual EvaluationReturn Evaluate(const world::World& world);
};

}  // namespace evaluation
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_EVALUATION_COLLISION_AGENTS_HPP_
