// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_EVALUATION_LTL_LABELS_RIGHT_OF_LABEL_FUNCTION_HPP_
#define MODULES_WORLD_EVALUATION_LTL_LABELS_RIGHT_OF_LABEL_FUNCTION_HPP_

#include "modules/world/evaluation/ltl/labels/multi_agent_label_function.hpp"
#include "modules/world/objects/object.hpp"

namespace modules {
namespace world {
namespace evaluation {

class RightOfLabelFunction : public MultiAgentLabelFunction {
  using MultiAgentLabelFunction::MultiAgentLabelFunction;
  bool EvaluateAgent(const world::ObservedWorld& observed_world,
                      const AgentPtr& other_agent) const override;
};

}  // namespace evaluation
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_EVALUATION_LTL_LABELS_RIGHT_OF_LABEL_FUNCTION_HPP_
