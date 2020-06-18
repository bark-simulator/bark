// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_EVALUATION_LTL_LABELS_LANE_CHANGE_LABEL_FUNCTION_HPP_
#define MODULES_WORLD_EVALUATION_LTL_LABELS_LANE_CHANGE_LABEL_FUNCTION_HPP_

#include <string>
#include <vector>

#include "modules/world/evaluation/ltl/labels/base_label_function.hpp"
#include "modules/world/objects/agent.hpp"
#include "modules/world/objects/object.hpp"

namespace modules {
namespace world {
namespace evaluation {

using modules::commons::transformation::FrenetPosition;
using modules::world::objects::AgentPtr;

class LaneChangeLabelFunction : public BaseLabelFunction {
 public:
  using BaseLabelFunction::BaseLabelFunction;
  std::vector<LabelMap::value_type> Evaluate(
      const world::ObservedWorld& observed_world) const override;
};

}  // namespace evaluation
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_EVALUATION_LTL_LABELS_LANE_CHANGE_LABEL_FUNCTION_HPP_
