// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_EVALUATION_EVALUATOR_GOAL_REACHED_HPP_
#define MODULES_WORLD_EVALUATION_EVALUATOR_GOAL_REACHED_HPP_

#include <memory>

#include "modules/world/evaluation/base_evaluator.hpp"
#include "modules/world/objects/agent.hpp"

namespace modules {
namespace world {
class World;
namespace evaluation {

class EvaluatorGoalReached : public BaseEvaluator {
 public:
  EvaluatorGoalReached() :
    agent_id_(-1) {}
  explicit EvaluatorGoalReached(const AgentId& agent_id) :
    agent_id_(agent_id) {}
  virtual ~EvaluatorGoalReached() {}

  virtual EvaluationReturn Evaluate(const world::World& world) {
    if (agent_id_ == -1) {
      int goal_reached_count = 0;
      for (auto& agent :  world.get_agents()) {
        if (agent.second->AtGoal()) {
          goal_reached_count += 1;
        }
      }
      if (goal_reached_count == world.get_agents().size())
        return true;
      return false;
    } else {
      auto agent_it = world.get_agents().find(agent_id_);
      if (agent_it != world.get_agents().end()) {
        return world.get_agents()[agent_id_]->AtGoal();
      } else {
        return false;
      }
    }
  }

 private:
  AgentId agent_id_;
};

}  // namespace evaluation
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_EVALUATION_EVALUATOR_GOAL_REACHED_HPP_
