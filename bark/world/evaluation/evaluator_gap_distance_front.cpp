// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/world/evaluation/evaluator_gap_distance_front.hpp"
#include <numeric>
#include "bark/commons/math/vector.hpp"

namespace bark {
namespace world {
class World;
namespace evaluation {

using bark::commons::transformation::FrenetPosition;
using world::AgentFrenetPair;

EvaluationReturn EvaluatorGapDistanceFront::Evaluate(
    const world::World& world) {
  auto cloned_world = world.Clone();
  if (world.GetAgent(agent_id_)) {
    return Evaluate(cloned_world->Observe({agent_id_})[0]);
  } else {
    double mean;
    if (!gap_vec_.empty()) {
      // still return mean value
      mean = bark::commons::math::CalculateMean(gap_vec_);
    } else {
      mean = nan("");
    }
    return mean;
  }
}

EvaluationReturn EvaluatorGapDistanceFront::Evaluate(
    const world::ObservedWorld& observed_world) {
  const auto& ego_agent = observed_world.GetEgoAgent();
  if (ego_agent) {
    AgentFrenetPair leading_vehicle_pair = observed_world.GetAgentInFront();
    if (leading_vehicle_pair.first) {
      // only add to vector if there is a vehicle in front
      FrenetPosition frenet_other = leading_vehicle_pair.second;
      double gap = frenet_other.lon - ego_agent->GetShape().front_dist_ -
                   leading_vehicle_pair.first->GetShape().rear_dist_;
      gap_vec_.push_back(gap);
    }
  }
  double mean;
  if (!gap_vec_.empty()) {
    mean = bark::commons::math::CalculateMean(gap_vec_);
  } else {
    mean = nan("");
  }
  return mean;
}

}  // namespace evaluation
}  // namespace world
}  // namespace bark
