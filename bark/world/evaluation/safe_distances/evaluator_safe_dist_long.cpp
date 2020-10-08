// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/world/observed_world.hpp"
#include "bark/world/evaluation/safe_distances/evaluator_safe_dist_long.hpp"

namespace bark {
namespace world {
namespace evaluation {

EvaluatorSafeDistLong::EvaluatorSafeDistLong(const bark::commons::ParamsPtr& params) :
    safe_dist_label_function("safe_dist_longitudinal", 
        params->GetBool("EvaluatorSafeDistLong::ToRear", "Include rear agent", true),
        params->GetReal("EvaluatorSafeDistLong::ReactionTime", "Reaction time", 100.0f),
        params->GetReal("EvaluatorSafeDistLong::MaxEgoDecceleration", "Maximum ego decceleration", 5.0f),
        params->GetReal("EvaluatorSafeDistLong::MaxOtherDecceleration", "Maximum other decceleration", 5.0f)) {}

EvaluationReturn EvaluatorSafeDistLong::Evaluate(
    const world::ObservedWorld& observed_world) {
  return safe_dist_label_function.Evaluate(observed_world).begin()->second;
}

}  // namespace evaluation
}  // namespace world
}  // namespace bark
