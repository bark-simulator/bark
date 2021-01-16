// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/world/evaluation/ltl/label_functions/rightmost_lane_label_function.hpp"

#include "bark/world/observed_world.hpp"

namespace bark {
namespace world {
namespace evaluation {

RightmostLaneLabelFunction::RightmostLaneLabelFunction(
    const std::string& label_str)
    : BaseLabelFunction(label_str) {}

LabelMap RightmostLaneLabelFunction::Evaluate(
    const world::ObservedWorld& observed_world) const {
  const auto& ego_pos = observed_world.CurrentEgoPosition();

  const auto right_lc = observed_world.GetRoadCorridor()
                            ->GetLeftRightLaneCorridor(ego_pos)
                            .second;
  bool is_rightmost_lane;
  if (right_lc) {
    is_rightmost_lane = false;
  } else {
    is_rightmost_lane = true;
  }
  return {{GetLabel(), is_rightmost_lane}};
}

}  // namespace evaluation
}  // namespace world
}  // namespace bark