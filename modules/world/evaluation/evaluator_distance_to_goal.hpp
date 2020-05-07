// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_EVALUATION_EVALUATOR_DISTANCE_TO_GOAL_HPP_
#define MODULES_WORLD_EVALUATION_EVALUATOR_DISTANCE_TO_GOAL_HPP_

#include <memory>
#include <limits>

#include "modules/world/evaluation/base_evaluator.hpp"
#include "modules/world/objects/agent.hpp"
#include "modules/world/observed_world.hpp"

namespace modules {
namespace world {
class World;
class ObservedWorld;
namespace evaluation {

class EvaluatorDistanceToGoal : public BaseEvaluator {
 public:
  EvaluatorDistanceToGoal() :
    agent_id_(std::numeric_limits<AgentId>::max()) {}
  explicit EvaluatorDistanceToGoal(const AgentId& agent_id) :
    agent_id_(agent_id) {}
  virtual ~EvaluatorDistanceToGoal() {}

  virtual EvaluationReturn Evaluate(const world::World& world) {
    const auto& agent = world.GetAgent(agent_id_);
    BARK_EXPECT_TRUE(bool(agent));
    return EvaluatorDistanceToGoal::DistanceToGoal(agent);
  }

  static float DistanceToGoal(
    const std::shared_ptr<const modules::world::objects::Agent>& agent) {
    const auto& goal_shape = agent->GetGoalDefinition()->GetShape();
    const auto& agent_pos = agent->GetCurrentPosition();
    float dist = modules::geometry::Distance(goal_shape, agent_pos);
    return dist;
  }

  virtual EvaluationReturn Evaluate(
    const world::ObservedWorld& observed_world) {
    const auto& ego_agent = observed_world.GetEgoAgent();
    BARK_EXPECT_TRUE(bool(ego_agent));
    return EvaluatorDistanceToGoal::DistanceToGoal(ego_agent);
  }

 private:
  AgentId agent_id_;
};

}  // namespace evaluation
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_EVALUATION_EVALUATOR_DISTANCE_TO_GOAL_HPP_
