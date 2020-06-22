// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "ego_beyond_point_label_function.hpp"
#include <unordered_map>
#include "bark/world/observed_world.hpp"

using modules::world::evaluation::LabelMap;

modules::world::evaluation::EgoBeyondPointLabelFunction::
    EgoBeyondPointLabelFunction(const std::string& label_str,
                                const modules::geometry::Point2d& beyond_point)
    : BaseLabelFunction(label_str), beyond_point_(beyond_point) {}
LabelMap modules::world::evaluation::EgoBeyondPointLabelFunction::Evaluate(
  const modules::world::ObservedWorld& observed_world) const {
  const auto ego_pos = observed_world.GetEgoAgent()->GetCurrentPosition();
  const auto lc = observed_world.GetLaneCorridor();
  if (lc) {
    FrenetPosition ego_frenet(ego_pos, lc->GetCenterLine());
    FrenetPosition point_frenet(beyond_point_, lc->GetCenterLine());
    return {{GetLabel(), ((ego_frenet.lon - point_frenet.lon) > 0)}};
  }
  return {{GetLabel(), false}};
}
const modules::geometry::Point2d&
modules::world::evaluation::EgoBeyondPointLabelFunction::GetBeyondPoint()
    const {
  return beyond_point_;
}
