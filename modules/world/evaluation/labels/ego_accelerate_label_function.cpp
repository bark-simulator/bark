// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/world/evaluation/labels/ego_accelerate_label_function.hpp"

#include <algorithm>
#include "modules/world/observed_world.hpp"

namespace modules {
namespace world {
namespace evaluation {

using modules::models::dynamic::StateDefinition;

EgoAccelerateLabelFunction::EgoAccelerateLabelFunction(
    const std::string& label_str, const double acc_thres,
    const size_t smooth_frame)
    : BaseLabelFunction(label_str),
      acc_thres_(acc_thres),
      smooth_frame_(smooth_frame) {}

std::vector<LabelMap::value_type> EgoAccelerateLabelFunction::Evaluate(
    const world::ObservedWorld& observed_world) const {
  bool accel = false;
  const auto ego = observed_world.GetEgoAgent();
  const auto& history = ego->GetStateInputHistory();
  if (history.size() > 2) {
    size_t frame_length = std::min(history.size(), smooth_frame_);
    const auto dx =
        history.back().first - (history.end() - 1 - frame_length)->first;
    const float dv = dx(StateDefinition::VEL_POSITION);
    const float dt = dx(StateDefinition::TIME_POSITION);
    const float avg_accel = dv / dt;
    accel = std::abs(avg_accel) < acc_thres_;
  }
  return {{GetLabel(), accel}};
}
}  // namespace evaluation
}  // namespace world
}  // namespace modules
