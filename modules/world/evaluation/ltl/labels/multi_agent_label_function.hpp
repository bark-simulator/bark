// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_EVALUATION_LTL_LABELS_MULTI_AGENT_LABEL_FUNCTION_HPP_
#define MODULES_WORLD_EVALUATION_LTL_LABELS_MULTI_AGENT_LABEL_FUNCTION_HPP_

#include <vector>

#include "modules/world/evaluation/ltl/labels/base_label_function.hpp"
#include "modules/world/objects/agent.hpp"

namespace modules {
namespace world {
namespace evaluation {

using objects::AgentPtr;

class MultiAgentLabelFunction : public BaseLabelFunction {
 public:
  using BaseLabelFunction::BaseLabelFunction;
  std::vector<LabelMap::value_type> Evaluate(
      const world::ObservedWorld& observed_world) const override;
  virtual bool EvaluateAgent(const world::ObservedWorld& observed_world,
                              const AgentPtr& other_agent) const = 0;
};

}  // namespace evaluation
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_EVALUATION_LTL_LABELS_MULTI_AGENT_LABEL_FUNCTION_HPP_
