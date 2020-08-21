// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_EVALUATION_EVALUATOR_CAPTURE_STATES_HPP_
#define BARK_WORLD_EVALUATION_EVALUATOR_CAPTURE_STATES_HPP_

#include <memory>

#include "bark/world/evaluation/base_evaluator.hpp"
#include "bark/world/observed_world.hpp"


namespace bark {
namespace world {

class World;
class ObservedWorld;
namespace evaluation {

class EvaluatorCaptureAgentStates : public BaseEvaluator {
 public:
  EvaluatorCaptureAgentStates() {}
  virtual ~EvaluatorCaptureAgentStates() {}

  EvaluationReturn Evaluate(const world::World& world) {
    std::map<std::string, State> states;
    for (const auto& agent : world.GetAgents()) {
      std::string idx = "state_" + std::to_string(agent.first);
      states[idx] = agent.second->GetCurrentState();
    }
    return states;
  }

  EvaluationReturn Evaluate(const world::ObservedWorld& world) {
    std::map<std::string, State> states;
    for (const auto& agent : world.GetAgents()) {
      std::string idx = "state_" + std::to_string(agent.first);
      states[idx] = agent.second->GetCurrentState();
    }
    return states;
  }
};

}  // namespace evaluation
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_EVALUATION_EVALUATOR_CAPTURE_STATES_HPP_
