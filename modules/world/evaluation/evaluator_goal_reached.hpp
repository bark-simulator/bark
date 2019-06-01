// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_EVALUATION_EVALUATOR_GOAL_REACHED_HPP_
#define MODULES_WORLD_EVALUATION_EVALUATOR_GOAL_REACHED_HPP_

#include <memory>

#include "modules/world/evaluation/base_evaluator.hpp"
#include "modules/world/objects/agent.hpp"

namespace modules
{
namespace world
{

class World;
namespace evaluation
{

class EvaluatorGoalReached : public BaseEvaluator
{
  public:
    EvaluatorGoalReached(const AgentId& agent_id)  {};
    virtual ~EvaluatorGoalReached() {};

    virtual float Evaluate(const world::World& world) const { return static_cast<float>(world.get_agents()[agent_id_]->AtGoal());}

  private: 
    AgentId agent_id_;
};

} // namespace collision
} // namespace world
} // namespace modules

#endif // MODULES_WORLD_EVALUATION_EVALUATOR_GOAL_REACHED_HPP_