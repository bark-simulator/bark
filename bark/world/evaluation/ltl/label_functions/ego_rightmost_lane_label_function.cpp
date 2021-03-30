// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/world/evaluation/ltl/label_functions/ego_rightmost_lane_label_function.hpp"

#include "bark/world/observed_world.hpp"

namespace bark {
namespace world {
namespace evaluation {

EgoRightmostLaneLabelFunction::EgoRightmostLaneLabelFunction(
    const std::string& label_str, const double distance_thres)
    : BaseLabelFunction(label_str), distance_thres_(distance_thres) {}

LabelMap EgoRightmostLaneLabelFunction::Evaluate(
    const world::ObservedWorld& observed_world) const {
  const auto& ego_agent = observed_world.GetEgoAgent();
  const auto& ego_pos = observed_world.CurrentEgoPosition();

  const auto right_lc = observed_world.GetRoadCorridor()
                            ->GetLeftRightLaneCorridor(ego_pos)
                            .second;
  bool is_rightmost_lane;
  if (right_lc) {
    const double dist_until_end =
        right_lc->LengthUntilEnd(ego_pos) - ego_agent->GetShape().front_dist_;
    if (dist_until_end > distance_thres_) {
      // there is a right lane that is not ending soon
      is_rightmost_lane = false;
    } else {
      is_rightmost_lane = true;
    }
  } else {
    is_rightmost_lane = true;
  }
  return {{GetLabel(), is_rightmost_lane}};
}

}  // namespace evaluation
}  // namespace world
}  // namespace bark