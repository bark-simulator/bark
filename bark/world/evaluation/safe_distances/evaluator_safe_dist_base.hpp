
// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_EVALUATION_SAFE_DISTANCES_EVALUATOR_SAFE_DIST_BASE_
#define BARK_WORLD_EVALUATION_SAFE_DISTANCES_EVALUATOR_SAFE_DIST_BASE_

#include "bark/world/evaluation/base_evaluator.hpp"

namespace bark {
namespace world {
class World;
namespace evaluation {

class EvaluatorSafeDistBase : public BaseEvaluator {
 public:
  EvaluatorSafeDistBase(const AgentId& agent_id) : violation_count_(0), agent_id_(agent_id) {};
  EvaluationReturn Evaluate(const world::World& world) override {
    const auto observed_world = world.Observe({agent_id_}).at(0);
    const auto eval_return = Evaluate(observed_world);
    return eval_return;
  }
  EvaluationReturn Evaluate(const world::ObservedWorld& observed_world) override {
    const auto safe_dist_check = CheckSafeDistance(observed_world);
    violation_count_ += safe_dist_check ? 0 : 1;
    return EvaluationReturn(violation_count_);
  };

  virtual bool CheckSafeDistance(const world::ObservedWorld& observed_world) const = 0;
 private:
  int violation_count_;
  const AgentId agent_id_;
};

}  // namespace evaluation
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_EVALUATION_SAFE_DISTANCES_EVALUATOR_SAFE_DIST_BASE_
