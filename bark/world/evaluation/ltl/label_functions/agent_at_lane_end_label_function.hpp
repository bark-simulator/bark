// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_EVALUATION_LTL_LABELS_AGENT_AT_LANE_END_LABEL_FUNCTION_HPP_
#define BARK_WORLD_EVALUATION_LTL_LABELS_AGENT_AT_LANE_END_LABEL_FUNCTION_HPP_

#include <string>

#include "bark/world/evaluation/ltl/label_functions/multi_agent_label_function.hpp"
#include "bark/world/objects/object.hpp"

namespace bark {
namespace world {
namespace evaluation {

class AgentAtLaneEndLabelFunction : public MultiAgentLabelFunction {
 public:
  AgentAtLaneEndLabelFunction(const std::string& string,
                              const double distance_thres = 50.0);
  bool EvaluateAgent(const world::ObservedWorld& observed_world,
                     const AgentPtr& other_agent) const override;
  double GetDistanceThres() const { return distance_thres_; }

 private:
  double distance_thres_;
};

}  // namespace evaluation
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_EVALUATION_LTL_LABELS_AGENT_AT_LANE_END_LABEL_FUNCTION_HPP_
