// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_EVALUATION_EVALUATOR_BEHAVIOR_EXPIRED_HPP_
#define BARK_WORLD_EVALUATION_EVALUATOR_BEHAVIOR_EXPIRED_HPP_

#include <limits>
#include <memory>

#include "bark/models/behavior/behavior_model.hpp"
#include "bark/world/evaluation/base_evaluator.hpp"
#include "bark/world/objects/agent.hpp"
#include "bark/world/observed_world.hpp"

namespace bark {
namespace world {
class World;
class ObservedWorld;
namespace evaluation {

using bark::models::behavior::BehaviorStatus;

class EvaluatorBehaviorExpired : public BaseEvaluator {
 public:
  EvaluatorBehaviorExpired() : agent_id_(std::numeric_limits<AgentId>::max()) {}
  explicit EvaluatorBehaviorExpired(const AgentId& agent_id)
      : agent_id_(agent_id) {}
  virtual ~EvaluatorBehaviorExpired() {}

  virtual EvaluationReturn Evaluate(const world::World& world) {
    const auto agent_ptr = world.GetAgent(agent_id_);
    if (agent_ptr) {
      if (agent_ptr->GetBehaviorStatus() == BehaviorStatus::EXPIRED) {
        return true;
      } else {
        return false;
      }
    } else {
      return true;
    }
  }

  virtual EvaluationReturn Evaluate(
      const world::ObservedWorld& observed_world) {
    const auto agent_ptr = observed_world.GetEgoAgent();
    if (agent_ptr) {
      if (agent_ptr->GetBehaviorStatus() == BehaviorStatus::EXPIRED) {
        return true;
      } else {
        return false;
      }
    } else {
      return true;
    }
  }

 private:
  AgentId agent_id_;
};

}  // namespace evaluation
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_EVALUATION_EVALUATOR_BEHAVIOR_EXPIRED_HPP_
