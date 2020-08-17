// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_EVALUATION_EVALUATOR_GOAL_REACHED_HPP_
#define BARK_WORLD_EVALUATION_EVALUATOR_GOAL_REACHED_HPP_

#include <limits>
#include <memory>

#include "bark/world/evaluation/base_evaluator.hpp"
#include "bark/world/objects/agent.hpp"
#include "bark/world/observed_world.hpp"

namespace bark {
namespace world {
class World;
class ObservedWorld;
namespace evaluation {

class EvaluatorGoalReached : public BaseEvaluator {
 public:
  EvaluatorGoalReached() : agent_id_(std::numeric_limits<AgentId>::max()) {}
  explicit EvaluatorGoalReached(const AgentId& agent_id)
      : agent_id_(agent_id) {}
  virtual ~EvaluatorGoalReached() {}

  virtual EvaluationReturn Evaluate(const world::World& world) {
    if (agent_id_ == std::numeric_limits<AgentId>::max()) {
      int goal_reached_count = 0;
      for (auto& agent : world.GetAgents()) {
        if (agent.second->AtGoal()) {
          goal_reached_count += 1;
        }
      }
      if (goal_reached_count == world.GetAgents().size()) return true;
      return false;
    } else {
      const auto agent_ptr = world.GetAgent(agent_id_);
      if (agent_ptr) {
        return agent_ptr->AtGoal();
      } else {
        return false;
      }
    }
  }

  virtual EvaluationReturn Evaluate(
      const world::ObservedWorld& observed_world) {
    const auto agent_ptr = observed_world.GetEgoAgent();
    if (agent_ptr) {
      return agent_ptr->AtGoal();
    } else {
      return false;
    }
  }

 private:
  AgentId agent_id_;
};

}  // namespace evaluation
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_EVALUATION_EVALUATOR_GOAL_REACHED_HPP_
