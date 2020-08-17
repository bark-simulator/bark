// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/world/evaluation/ltl/label_functions/ego_accelerate_label_function.hpp"

#include <algorithm>
#include "bark/world/observed_world.hpp"

namespace bark {
namespace world {
namespace evaluation {

using bark::models::dynamic::StateDefinition;

EgoAccelerateLabelFunction::EgoAccelerateLabelFunction(
    const std::string& label_str, double acc_thres)
    : BaseLabelFunction(label_str), acc_thres_(acc_thres) {}

LabelMap EgoAccelerateLabelFunction::Evaluate(
    const world::ObservedWorld& observed_world) const {
  bool accel = false;
  const auto ego = observed_world.GetEgoAgent();
  const auto& history = ego->GetStateInputHistory();
  if (history.size() > 2) {
    const auto dx = (history.end() - 1)->first - (history.end() - 2)->first;
    const float dv = dx(StateDefinition::VEL_POSITION);
    const float dt = dx(StateDefinition::TIME_POSITION);
    const float avg_accel = dv / dt;
    accel = avg_accel >= acc_thres_;
  }
  return {{GetLabel(), accel}};
}
}  // namespace evaluation
}  // namespace world
}  // namespace bark
