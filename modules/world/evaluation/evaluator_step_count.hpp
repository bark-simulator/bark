// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_EVALUATION_EVALUATOR_STEP_COUNT_HPP_
#define MODULES_WORLD_EVALUATION_EVALUATOR_STEP_COUNT_HPP_

#include <memory>

#include "modules/world/evaluation/base_evaluator.hpp"
#include "modules/world/observed_world.hpp"

namespace modules {
namespace world {

class World;
class ObservedWorld;
namespace evaluation {

class EvaluatorStepCount : public BaseEvaluator {
 public:
  EvaluatorStepCount() : steps_(0) { }
  virtual ~EvaluatorStepCount() { }

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
}  // namespace modules

#endif  // MODULES_WORLD_EVALUATION_EVALUATOR_STEP_COUNT_HPP_
