// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "ego_beyond_point_label_evaluator.hpp"
#include <unordered_map>
#include "modules/world/observed_world.hpp"

using modules::world::evaluation::LabelMap;


modules::world::evaluation::EgoBeyondPointLabelEvaluator::
EgoBeyondPointLabelEvaluator(const std::string& label_str,
                             const modules::geometry::Point2d& beyond_point)
    : BaseLabelEvaluator(label_str), beyond_point_(beyond_point) {}
std::vector<LabelMap::value_type>
modules::world::evaluation::EgoBeyondPointLabelEvaluator::Evaluate(
    const modules::world::ObservedWorld& observed_world) const {
  const auto ego_pos = observed_world.GetEgoAgent()->GetCurrentPosition();
  const auto lc = observed_world.GetLaneCorridor();
  if (lc) {
    FrenetPosition ego_frenet(ego_pos, lc->GetCenterLine());
    FrenetPosition point_frenet(beyond_point_, lc->GetCenterLine());
    return {{GetLabel(), ((ego_frenet.lon - point_frenet.lon) > 0) }};
  }
  return {{GetLabel(), false }};
}
const modules::geometry::Point2d&
modules::world::evaluation::EgoBeyondPointLabelEvaluator::GetBeyondPoint()
    const {
  return beyond_point_;
}
