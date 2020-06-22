// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_EVALUATION_LTL_EVALUATOR_RIGHT_OVERTAKE_HPP_
#define MODULES_WORLD_EVALUATION_LTL_EVALUATOR_RIGHT_OVERTAKE_HPP_

#include "bark/world/evaluation/ltl/evaluator_ltl.hpp"

namespace modules {
namespace world {
namespace evaluation {

class EvaluatorRightOvertake : public EvaluatorLTL {
 public:
  explicit EvaluatorRightOvertake(AgentId agent_id)
      : EvaluatorLTL(agent_id, formula_, labels_) {}

  static const char formula_[];
  static const LabelFunctions labels_;
};

class EvaluatorRightOvertakeAssumption : public EvaluatorLTL {
 public:
  explicit EvaluatorRightOvertakeAssumption(AgentId agent_id)
      : EvaluatorLTL(agent_id, formula_, labels_) {}

  static const char formula_[];
  static const LabelFunctions labels_;
};

}  // namespace evaluation
}  // namespace world
}  // namespace modules
#endif  // MODULES_WORLD_EVALUATION_LTL_EVALUATOR_RIGHT_OVERTAKE_HPP_
