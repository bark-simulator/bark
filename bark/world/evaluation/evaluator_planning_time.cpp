// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/world/evaluation/evaluator_planning_time.hpp"
#include <numeric>
#include "bark/commons/math/vector.hpp"

namespace bark {
namespace world {
class World;
namespace evaluation {
using bark::models::behavior::BehaviorModelPtr;

EvaluationReturn EvaluatorPlanningTime::Evaluate(const world::World& world) {
  auto cloned_world = world.Clone();
  if (world.GetAgent(agent_id_)) {
    return Evaluate(cloned_world->Observe({agent_id_})[0]);
  } else {
    double mean;
    if (!planning_times_.empty()) {
      mean = bark::commons::math::CalculateMean(planning_times_);
    } else {
      // if empty, return nan to indicate that it could not be calculated
      mean = nan("");
    }
    return mean;
  }
}

EvaluationReturn EvaluatorPlanningTime::Evaluate(
    const world::ObservedWorld& observed_world) {
  auto ego_agent = observed_world.GetEgoAgent();
  BehaviorModelPtr model = ego_agent->GetBehaviorModel();
  double planning_time = model->GetLastSolutionTime();
  planning_times_.push_back(planning_time);
  double mean = bark::commons::math::CalculateMean(planning_times_);
  return mean;
}

}  // namespace evaluation
}  // namespace world
}  // namespace bark
