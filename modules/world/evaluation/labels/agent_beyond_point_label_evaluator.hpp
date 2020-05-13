// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_EVALUATION_LABELS_AGENT_BEYOND_POINT_LABEL_EVALUATOR_HPP_
#define MODULES_WORLD_EVALUATION_LABELS_AGENT_BEYOND_POINT_LABEL_EVALUATOR_HPP_

#include <string>

#include "modules/world/evaluation/labels/multi_agent_label_evaluator.hpp"
#include "modules/world/objects/object.hpp"
#include "modules/world/opendrive/commons.hpp"

namespace modules {
namespace world {
namespace evaluation {
using opendrive::XodrLaneId;
using geometry::Point2d;
class AgentBeyondPointLabelEvaluator : public MultiAgentLabelEvaluator {
 public:
  AgentBeyondPointLabelEvaluator(const std::string& string, Point2d beyond_point);
  bool evaluate_agent(const world::ObservedWorld& observed_world,
                      const AgentPtr& other_agent) const override;
  const Point2d& GetBeyondPoint() const;

 private:
  Point2d beyond_point_;
};

}  // namespace evaluation
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_EVALUATION_LABELS_AGENT_BEYOND_POINT_LABEL_EVALUATOR_HPP_
