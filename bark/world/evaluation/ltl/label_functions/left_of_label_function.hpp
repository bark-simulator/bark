// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_EVALUATION_LTL_LABELS_LEFT_OF_LABEL_FUNCTION_HPP_
#define BARK_WORLD_EVALUATION_LTL_LABELS_LEFT_OF_LABEL_FUNCTION_HPP_

#include "bark/world/evaluation/ltl/label_functions/multi_agent_label_function.hpp"
#include "bark/world/objects/object.hpp"

namespace bark {
namespace world {
namespace evaluation {

class LeftOfLabelFunction : public MultiAgentLabelFunction {
  using MultiAgentLabelFunction::MultiAgentLabelFunction;

 public:
  bool EvaluateAgent(const world::ObservedWorld& observed_world,
                     const AgentPtr& other_agent) const override;
};

}  // namespace evaluation
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_EVALUATION_LTL_LABELS_LEFT_OF_LABEL_FUNCTION_HPP_
