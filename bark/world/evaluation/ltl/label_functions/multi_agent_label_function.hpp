// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_EVALUATION_LTL_LABELS_MULTI_AGENT_LABEL_FUNCTION_HPP_
#define BARK_WORLD_EVALUATION_LTL_LABELS_MULTI_AGENT_LABEL_FUNCTION_HPP_

#include <vector>

#include "bark/world/evaluation/ltl/label_functions/base_label_function.hpp"
#include "bark/world/objects/agent.hpp"

namespace bark {
namespace world {
namespace evaluation {

using objects::AgentPtr;

class MultiAgentLabelFunction : public BaseLabelFunction {
 public:
  using BaseLabelFunction::BaseLabelFunction;
  LabelMap Evaluate(const world::ObservedWorld& observed_world) const override;
  virtual bool EvaluateAgent(const world::ObservedWorld& observed_world,
                             const AgentPtr& other_agent) const = 0;
};

}  // namespace evaluation
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_EVALUATION_LTL_LABELS_MULTI_AGENT_LABEL_FUNCTION_HPP_
