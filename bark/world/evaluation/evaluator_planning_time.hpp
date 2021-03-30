// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_EVALUATION_EVALUATOR_PLANNING_TIME_HPP_
#define BARK_WORLD_EVALUATION_EVALUATOR_PLANNING_TIME_HPP_

#include <limits>
#include <memory>

#include "bark/world/evaluation/base_evaluator.hpp"
#include "bark/world/observed_world.hpp"
#include "bark/world/world.hpp"

namespace bark {
namespace world {
namespace evaluation {

class EvaluatorPlanningTime : public BaseEvaluator {
 public:
  EvaluatorPlanningTime()
      : agent_id_(std::numeric_limits<AgentId>::max()), planning_times_() {}
  explicit EvaluatorPlanningTime(const AgentId& agent_id)
      : agent_id_(agent_id), planning_times_() {}
  virtual ~EvaluatorPlanningTime() {}
  virtual EvaluationReturn Evaluate(const world::World& world);
  virtual EvaluationReturn Evaluate(const world::ObservedWorld& observed_world);
 private:
  AgentId agent_id_;
  std::vector<double> planning_times_;
};

}  // namespace evaluation
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_EVALUATION_EVALUATOR_PLANNING_TIME_HPP_
