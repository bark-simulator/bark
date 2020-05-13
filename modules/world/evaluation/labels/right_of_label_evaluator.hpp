// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_EVALUATION_RIGHT_OF_LABEL_EVALUATOR_HPP_
#define MODULES_WORLD_EVALUATION_RIGHT_OF_LABEL_EVALUATOR_HPP_

#include "modules/world/evaluation/labels/multi_agent_label_evaluator.hpp"
#include "modules/world/objects/object.hpp"

namespace modules {
namespace world {
namespace evaluation {

class RightOfLabelEvaluator : public MultiAgentLabelEvaluator {
  using MultiAgentLabelEvaluator::MultiAgentLabelEvaluator;
  bool evaluate_agent(const world::ObservedWorld& observed_world, const AgentPtr &other_agent) const override;
};

}  // namespace evaluation
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_EVALUATION_RIGHT_OF_LABEL_EVALUATOR_HPP_
