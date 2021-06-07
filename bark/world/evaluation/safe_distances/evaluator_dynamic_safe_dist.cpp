// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/world/observed_world.hpp"
#include "bark/world/evaluation/safe_distances/evaluator_dynamic_safe_dist.hpp"

namespace bark {
namespace world {
namespace evaluation {

EvaluatorDynamicSafeDist::EvaluatorDynamicSafeDist(const bark::commons::ParamsPtr& params, const AgentId& agent_id) :
    EvaluatorSafeDistBase(agent_id),
    safe_dist_label_function("safe_dist_longitudinal", 
        params->GetBool("EvaluatorDynamicSafeDist::ToRear", "Include rear agent", true),
        params->GetReal("EvaluatorDynamicSafeDist::ReactionTimeEgo", "Reaction time ego", 0.2f),
        params->GetReal("EvaluatorDynamicSafeDist::ReactionTimeOthers", "Reaction time others", 0.8f),
        params->GetReal("EvaluatorDynamicSafeDist::MaxEgoDecceleration", "Maximum ego decceleration", 5.0f),
        params->GetReal("EvaluatorDynamicSafeDist::MaxOtherDecceleration", "Maximum other decceleration", 5.0f),
        params->GetBool("EvaluatorDynamicSafeDist::ConsiderCrossingCorridors", "If true agens in other lane corridors are considered", false),
        params->GetInt("EvaluatorDynamicSafeDist::MaxAgentssCrossingCorridors", "How many nearest agents are lookup in other corridors", 4),
        params->GetBool("EvaluatorDynamicSafeDist::UseFracParamFromWorld", "True, if lateral distance threshold is overriden from world param", true),
        params->GetReal("EvaluatorDynamicSafeDist::LateralDistanceThreshold", "Lateral distance between vehicles to consider them as unsafe", 1.0),
        params->GetBool("EvaluatorDynamicSafeDist::CheckLateralSafeDist", "If lateral safe dist should be checked", false)) {}

bool EvaluatorDynamicSafeDist::CheckSafeDistance(
    const world::ObservedWorld& observed_world) {
  return safe_dist_label_function.Evaluate(observed_world).begin()->second;
}

}  // namespace evaluation
}  // namespace world
}  // namespace bark
