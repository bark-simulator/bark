// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_EVALUATION_EVALUATOR_NUMBER_OF_AGENTS_HPP_
#define BARK_WORLD_EVALUATION_EVALUATOR_NUMBER_OF_AGENTS_HPP_

#include <limits>
#include <memory>

#include "bark/world/evaluation/base_evaluator.hpp"
#include "bark/world/observed_world.hpp"
#include "bark/world/world.hpp"

namespace bark {
namespace world {
namespace evaluation {

class EvaluatorNumberOfAgents : public BaseEvaluator {
 public:
  EvaluatorNumberOfAgents()
      : agent_id_(std::numeric_limits<AgentId>::max()), agent_ids_() {}
  explicit EvaluatorNumberOfAgents(const AgentId& agent_id)
      : agent_id_(agent_id), agent_ids_() {}
  virtual ~EvaluatorNumberOfAgents() {}
  virtual EvaluationReturn Evaluate(const world::World& world);
  virtual EvaluationReturn Evaluate(const world::ObservedWorld& observed_world);

 private:
  AgentId agent_id_;
  std::set<AgentId> agent_ids_;
};

}  // namespace evaluation
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_EVALUATION_EVALUATOR_NUMBER_OF_AGENTS_HPP_
