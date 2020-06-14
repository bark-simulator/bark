// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_EVALUATION_EVALUATOR_BEING_OVERTAKEN_HPP_
#define MODULES_WORLD_EVALUATION_EVALUATOR_BEING_OVERTAKEN_HPP_

#include "modules/world/evaluation/evaluator_ltl.hpp"

namespace modules {
namespace world {
namespace evaluation {

class EvaluatorBeingOvertaken : public EvaluatorLTL {
 public:
  explicit EvaluatorBeingOvertaken(AgentId agent_id)
      : EvaluatorLTL(agent_id, formula_) {}
  EvaluationReturn Evaluate(const world::World& world) override {
    auto cloned_world = world.Clone();
    cloned_world->AddLabels(labels_);
    return EvaluatorLTL::Evaluate(*cloned_world);
  }

  static const char formula_[];
  static const LabelFunctions labels_;
};

class EvaluatorBeingOvertakenAssumption : public EvaluatorLTL {
 public:
  explicit EvaluatorBeingOvertakenAssumption(AgentId agent_id)
      : EvaluatorLTL(agent_id, formula_) {}
  EvaluationReturn Evaluate(const world::World& world) override {
    auto cloned_world = world.Clone();
    cloned_world->AddLabels(labels_);
    return EvaluatorLTL::Evaluate(*cloned_world);
  }

  static const char formula_[];
  static const LabelFunctions labels_;
};

}  // namespace evaluation
}  // namespace world
}  // namespace modules
#endif  // MODULES_WORLD_EVALUATION_EVALUATOR_BEING_OVERTAKEN_HPP_
