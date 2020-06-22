// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_EVALUATION_LTL_LABELS_REL_SPEED_LABEL_FUNCTION_HPP_
#define MODULES_WORLD_EVALUATION_LTL_LABELS_REL_SPEED_LABEL_FUNCTION_HPP_

#include <string>

#include "bark/world/evaluation/ltl/labels/multi_agent_label_function.hpp"
#include "bark/world/objects/object.hpp"

namespace modules {
namespace world {
namespace evaluation {

// TRUE if relative speed between two agents is GREATER EQUAL than the
// threshold
class RelSpeedLabelFunction : public MultiAgentLabelFunction {
 public:
  RelSpeedLabelFunction(const std::string& string, double rel_speed_thres);
  bool EvaluateAgent(const world::ObservedWorld& observed_world,
                     const AgentPtr& other_agent) const override;

 private:
  const double rel_speed_thres_;
};

}  // namespace evaluation
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_EVALUATION_LTL_LABELS_REL_SPEED_LABEL_FUNCTION_HPP_
