// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_EVALUATION_SAFE_DISTANCES_EVALUATOR_SAFE_DIST_LONG_
#define BARK_WORLD_EVALUATION_SAFE_DISTANCES_EVALUATOR_SAFE_DIST_LONG_

#include "bark/world/evaluation/safe_distances/evaluator_safe_dist_base.hpp"
#include "bark/world/evaluation/ltl/label_functions/safe_distance_label_function.hpp"

namespace bark {
namespace world {
class World;
namespace evaluation {

class EvaluatorDynamicSafeDistLong : public EvaluatorSafeDistBase {
 public:
  EvaluatorDynamicSafeDistLong(const bark::commons::ParamsPtr& params, const AgentId& agent_id);
  bool CheckSafeDistance(const world::ObservedWorld& observed_world) override;
 private:
  SafeDistanceLabelFunction safe_dist_label_function;
};

}  // namespace evaluation
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_EVALUATION_SAFE_DISTANCES_EVALUATOR_SAFE_DIST_LONG_
