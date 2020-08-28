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
#include <string>
#include <map>

#include "bark/world/evaluation/base_evaluator.hpp"
#include "bark/world/observed_world.hpp"


namespace bark {
namespace world {

class World;
class ObservedWorld;
namespace evaluation {

using AgentStateMap = std::map<std::string, State>;

template<typename T>
AgentStateMap CaptureAgentStates(const T& world) {
  AgentStateMap state_map;
  for (const auto& agent : world.GetAgents()) {
    std::string idx = "state_" + std::to_string(agent.first);
    state_map[idx] = agent.second->GetCurrentState();
  }
  return state_map;
}


}  // namespace evaluation
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_EVALUATION_EVALUATOR_CAPTURE_STATES_HPP_
