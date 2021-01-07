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
  if (ego_agent) {
    double vel = ego_agent->GetCurrentState()[StateDefinition::VEL_POSITION];
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

EvaluationReturn EvaluatorVelocity::Evaluate(
    const world::ObservedWorld& observed_world) {
  double vel = observed_world.CurrentEgoState()[StateDefinition::VEL_POSITION];
  vel_vec_.push_back(vel);
  double mean = bark::commons::math::CalculateMean(vel_vec_);
  return mean;
}

}  // namespace evaluation
}  // namespace world
}  // namespace bark
