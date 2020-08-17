// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_EVALUATION_EVALUATOR_STEP_COUNT_HPP_
#define BARK_WORLD_EVALUATION_EVALUATOR_STEP_COUNT_HPP_

#include <memory>

#include "bark/world/evaluation/base_evaluator.hpp"
#include "bark/world/observed_world.hpp"

namespace bark {
namespace world {

class World;
class ObservedWorld;
namespace evaluation {

class EvaluatorStepCount : public BaseEvaluator {
 public:
  EvaluatorStepCount() : steps_(0) {}
  virtual ~EvaluatorStepCount() {}

  EvaluationReturn Evaluate(const world::World& world) {
    this->steps_++;
    return this->steps_;
  }

  EvaluationReturn Evaluate(const world::ObservedWorld& world) {
    this->steps_++;
    return this->steps_;
  }

 private:
  int steps_;
};

}  // namespace evaluation
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_EVALUATION_EVALUATOR_STEP_COUNT_HPP_
