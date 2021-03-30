// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_EVALUATION_LTL_LABELS_BELOW_SPEED_LIMIT_LABEL_FUNCTION_HPP_
#define BARK_WORLD_EVALUATION_LTL_LABELS_BELOW_SPEED_LIMIT_LABEL_FUNCTION_HPP_

#include <string>

#include "bark/world/evaluation/ltl/label_functions/multi_agent_label_function.hpp"

#include "bark/world/objects/object.hpp"

namespace bark {
namespace world {
namespace evaluation {

class BelowSpeedLimitLabelFunction : public MultiAgentLabelFunction {
 public:
  BelowSpeedLimitLabelFunction(const std::string& string,
                               const double velocity_thres);
  bool EvaluateAgent(const world::ObservedWorld& observed_world,
                const AgentPtr& other_agent) const override;
  double GetVelocityThres() const { return velocity_thres_; }

 private:
  double velocity_thres_;
};

}  // namespace evaluation
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_EVALUATION_LTL_LABELS_BELOW_SPEED_LIMIT_LABEL_FUNCTION_HPP_
