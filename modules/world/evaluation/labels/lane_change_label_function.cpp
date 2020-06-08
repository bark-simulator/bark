// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/world/evaluation/labels/lane_change_label_function.hpp"

#include "modules/geometry/polygon.hpp"
#include "modules/world/observed_world.hpp"

namespace modules {
namespace world {
namespace evaluation {

using modules::geometry::Collide;
using modules::geometry::Polygon;
using modules::models::dynamic::StateDefinition;

std::vector<LabelMap::value_type> LaneChangeLabelFunction::Evaluate(
    const world::ObservedWorld& observed_world) const {
  bool lane_change = false;
  const auto lc = observed_world.GetLaneCorridor();
  const auto ego = observed_world.GetEgoAgent();
  if (ego->GetStateInputHistory().size() >= 2 && lc) {
    const State prev_state = (ego->GetStateInputHistory().end() - 2)->first;
    const geometry::LinePoint prev_pos(prev_state(StateDefinition::X_POSITION),
                                       prev_state(StateDefinition::Y_POSITION));
    const auto& prev_lc =
        ego->GetRoadCorridor()->GetNearestLaneCorridor(prev_pos);
    const double vehicle_area = ego->GetShape().CalculateArea();
    const double current_overlap =
        CalculateOverlapArea(lc, ego, ego->GetCurrentState()) / vehicle_area;
    const double prev_overlap =
        CalculateOverlapArea(prev_lc, ego, prev_state) / vehicle_area;
    lane_change = (prev_overlap < 0.85 && current_overlap >= 0.85);
  }
  return {{GetLabel(), lane_change}};
}
double LaneChangeLabelFunction::CalculateOverlapArea(
    const LaneCorridorPtr& lc, const std::shared_ptr<const Agent>& ego,
    const State& state) const {
  return IntersectionArea(ego->GetPolygonFromState(state),
                          lc->GetMergedPolygon());
}
}  // namespace evaluation
}  // namespace world
}  // namespace modules
