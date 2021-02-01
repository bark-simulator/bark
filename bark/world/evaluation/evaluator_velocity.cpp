// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/world/evaluation/evaluator_velocity.hpp"
#include <numeric>
#include "bark/commons/math/vector.hpp"

namespace bark {
namespace world {
class World;
namespace evaluation {

using bark::models::dynamic::StateDefinition;

EvaluationReturn EvaluatorVelocity::Evaluate(const world::World& world) {
  auto ego_agent = world.GetAgent(this->agent_id_);
  double mean = CalculateMeanVelocity(ego_agent);
  return mean;
}

EvaluationReturn EvaluatorVelocity::Evaluate(
    const world::ObservedWorld& observed_world) {
  const auto& ego_agent = observed_world.GetEgoAgent();
  double mean = CalculateMeanVelocity(ego_agent);
  return mean;
}

double EvaluatorVelocity::CalculateMeanVelocity(
    const std::shared_ptr<const bark::world::objects::Agent>& agent) {
  if (agent) {
    double vel = agent->GetCurrentState()[StateDefinition::VEL_POSITION];
    vel_vec_.push_back(vel);
  }
  double mean;
  if (!vel_vec_.empty()) {
    mean = bark::commons::math::CalculateMean(vel_vec_);
  } else {
    mean = nan("");
  }
  return mean;
}

}  // namespace evaluation
}  // namespace world
}  // namespace bark
