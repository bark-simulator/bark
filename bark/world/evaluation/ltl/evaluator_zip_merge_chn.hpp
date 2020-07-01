// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_EVALUATION_LTL_EVALUATOR_ZIP_MERGE_CHN_HPP_
#define BARK_WORLD_EVALUATION_LTL_EVALUATOR_ZIP_MERGE_CHN_HPP_

#include "bark/world/evaluation/ltl/evaluator_ltl.hpp"

namespace bark {
namespace world {
namespace evaluation {

class EvaluatorZipMergeChn : public EvaluatorLTL {
 public:
  explicit EvaluatorZipMergeChn(AgentId agent_id)
      : EvaluatorLTL(agent_id, formula_, labels_) {}

  static const char formula_[];
  static const LabelFunctions labels_;
};

}  // namespace evaluation
}  // namespace world
}  // namespace bark
#endif  // BARK_WORLD_EVALUATION_LTL_EVALUATOR_ZIP_MERGE_CHN_HPP_
