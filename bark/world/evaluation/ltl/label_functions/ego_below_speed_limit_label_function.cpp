// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/world/evaluation/ltl/label_functions/ego_below_speed_limit_label_function.hpp"

#include "bark/world/observed_world.hpp"

using bark::models::dynamic::StateDefinition;

namespace bark {
namespace world {
namespace evaluation {

EgoBelowSpeedLimitLabelFunction::EgoBelowSpeedLimitLabelFunction(
    const std::string& string, const double velocity_thres)
    : BaseLabelFunction(string), velocity_thres_(velocity_thres) {
  assert(velocity_thres_ >= 0.0);
}

LabelMap EgoBelowSpeedLimitLabelFunction::Evaluate(
    const bark::world::ObservedWorld& observed_world) const {
  const auto& ego_vehicle_state = observed_world.CurrentEgoState();
  const double vel = ego_vehicle_state(StateDefinition::VEL_POSITION);
  return {{GetLabel(), vel < velocity_thres_}};
}

}  // namespace evaluation
}  // namespace world
}  // namespace bark