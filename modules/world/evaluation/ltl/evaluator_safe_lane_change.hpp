// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_EVALUATION_LTL_EVALUATOR_SAFE_LANE_CHANGE_HPP_
#define MODULES_WORLD_EVALUATION_LTL_EVALUATOR_SAFE_LANE_CHANGE_HPP_

#include "modules/world/evaluation/ltl/evaluator_ltl.hpp"

namespace modules {
namespace world {
namespace evaluation {

class EvaluatorSafeLaneChange : public EvaluatorLTL {
 public:
  explicit EvaluatorSafeLaneChange(AgentId agent_id)
      : EvaluatorLTL(agent_id, formula_, labels_) {}

  static const char formula_[];
  static const LabelFunctions labels_;

  constexpr static double reaction_time = 1.0;
  // From "A formally verified motion planner for autonomous vehicles"
  constexpr static double decel_rear = -7.84;
  constexpr static double decel_ego = -7.84;
};

class EvaluatorSafeLaneChangeAssumption : public EvaluatorLTL {
 public:
  explicit EvaluatorSafeLaneChangeAssumption(AgentId agent_id)
      : EvaluatorLTL(agent_id, formula_, labels_) {}

  static const char formula_[];
  static const LabelFunctions labels_;
};

}  // namespace evaluation
}  // namespace world
}  // namespace modules
#endif  // MODULES_WORLD_EVALUATION_LTL_EVALUATOR_SAFE_LANE_CHANGE_HPP_
