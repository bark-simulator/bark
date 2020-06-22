// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/world/evaluation/ltl/labels/lane_change_label_function.hpp"

#include "bark/world/observed_world.hpp"

namespace modules {
namespace world {
namespace evaluation {

using modules::geometry::Collide;
using modules::models::dynamic::State;
using modules::models::dynamic::StateDefinition;

LabelMap LaneChangeLabelFunction::Evaluate(
  const world::ObservedWorld& observed_world) const {
  bool lane_change = false;
  const auto lc = observed_world.GetLaneCorridor();
  const auto ego = observed_world.GetEgoAgent();
  if (ego->GetStateInputHistory().size() >= 2 && lc) {
    const State prev_state = (ego->GetStateInputHistory().end() - 2)->first;
    const geometry::LinePoint prev_pos(prev_state(StateDefinition::X_POSITION),
                                       prev_state(StateDefinition::Y_POSITION));
    const auto current_pos = observed_world.GetEgoAgent()->GetCurrentPosition();
    geometry::Line line;
    line.AddPoint(prev_pos);
    line.AddPoint(current_pos);
    lane_change = Collide(line, lc->GetLeftBoundary());
    lane_change = lane_change || Collide(line, lc->GetRightBoundary());
  }
  return {{GetLabel(), lane_change}};
}
}  // namespace evaluation
}  // namespace world
}  // namespace modules
