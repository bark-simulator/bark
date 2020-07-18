// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "ego_beyond_point_label_function.hpp"
#include <unordered_map>
#include "bark/world/observed_world.hpp"

using bark::world::evaluation::LabelMap;

bark::world::evaluation::EgoBeyondPointLabelFunction::
    EgoBeyondPointLabelFunction(const std::string& label_str,
                                const bark::geometry::Point2d& beyond_point)
    : BaseLabelFunction(label_str), beyond_point_(beyond_point) {}
LabelMap bark::world::evaluation::EgoBeyondPointLabelFunction::Evaluate(
    const bark::world::ObservedWorld& observed_world) const {
  const auto ego_pos = observed_world.GetEgoAgent()->GetCurrentPosition();
  const auto lc = observed_world.GetLaneCorridor();
  if (lc) {
    FrenetPosition ego_frenet(ego_pos, lc->GetCenterLine());
    FrenetPosition point_frenet(beyond_point_, lc->GetCenterLine());
    return {{GetLabel(), ((ego_frenet.lon - point_frenet.lon) > 0)}};
  }
  return {{GetLabel(), false}};
}
const bark::geometry::Point2d&
bark::world::evaluation::EgoBeyondPointLabelFunction::GetBeyondPoint() const {
  return beyond_point_;
}
