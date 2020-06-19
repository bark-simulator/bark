// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_EVALUATION_LABELS_AGENT_BEYOND_POINT_LABEL_FUNCTION_HPP_
#define MODULES_WORLD_EVALUATION_LABELS_AGENT_BEYOND_POINT_LABEL_FUNCTION_HPP_

#include <string>

#include "bark/world/evaluation/labels/multi_agent_label_function.hpp"
#include "bark/world/objects/object.hpp"
#include "bark/world/opendrive/commons.hpp"

namespace modules {
namespace world {
namespace evaluation {
using geometry::Point2d;
using opendrive::XodrLaneId;
class AgentBeyondPointLabelFunction : public MultiAgentLabelFunction {
 public:
  AgentBeyondPointLabelFunction(const std::string& string,
                                Point2d beyond_point);
  bool EvaluateAgent(const world::ObservedWorld& observed_world,
                      const AgentPtr& other_agent) const override;
  const Point2d& GetBeyondPoint() const;

 private:
  Point2d beyond_point_;
};

}  // namespace evaluation
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_EVALUATION_LABELS_AGENT_BEYOND_POINT_LABEL_FUNCTION_HPP_
