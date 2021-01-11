// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/world/evaluation/evaluator_number_of_agents.hpp"
#include <numeric>
#include "bark/commons/math/vector.hpp"

namespace bark {
namespace world {
class World;
namespace evaluation {

EvaluationReturn EvaluatorNumberOfAgents::Evaluate(const world::World& world) {
  auto cloned_world = world.Clone();
  if (world.GetAgent(agent_id_)) {
    return Evaluate(cloned_world->Observe({agent_id_})[0]);
  } else {
    return static_cast<int>(agent_ids_.size());
  }
}

EvaluationReturn EvaluatorNumberOfAgents::Evaluate(
    const world::ObservedWorld& observed_world) {
  auto agents = observed_world.GetAgents();
  for (auto it = agents.begin(); it != agents.end(); ++it) {
    agent_ids_.push_back(it->first);
  }

  std::sort(agent_ids_.begin(), agent_ids_.end());
  agent_ids_.erase(std::unique(agent_ids_.begin(), agent_ids_.end()),
                   agent_ids_.end());

  return static_cast<int>(agent_ids_.size());
}

}  // namespace evaluation
}  // namespace world
}  // namespace bark
