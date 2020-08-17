// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/world/map/road_corridor.hpp"
#include <memory>
#include <utility>
#include "bark/commons/transformation/frenet.hpp"

namespace bark {
namespace world {
namespace map {

std::pair<LaneCorridorPtr, LaneCorridorPtr>
RoadCorridor::GetLeftRightLaneCorridor(const Point2d& pt) const {
  LaneCorridorPtr current_lane_corr = GetCurrentLaneCorridor(pt);
  if (!current_lane_corr) {
    return std::make_pair(LaneCorridorPtr(nullptr), LaneCorridorPtr(nullptr));
  }
  LanePtr current_lane = current_lane_corr->GetCurrentLane(pt);
  std::shared_ptr<Lane> left_lane, right_lane;
  if (current_lane) {
    left_lane = current_lane_corr->GetCurrentLane(pt)->GetLeftLane();
    right_lane = current_lane_corr->GetCurrentLane(pt)->GetRightLane();
  }
  LaneId left_lane_id = 10000000;
  LaneId right_lane_id = 10000000;
  if (left_lane) left_lane_id = left_lane->GetId();
  if (right_lane) right_lane_id = right_lane->GetId();
  return std::make_pair(GetLaneCorridor(left_lane_id),
                        GetLaneCorridor(right_lane_id));
}

LaneCorridorPtr RoadCorridor::GetNearestLaneCorridor(const Point2d& pt) const {
  using bark::commons::transformation::FrenetPosition;
  auto lc = GetCurrentLaneCorridor(pt);
  if (!lc) {
    double min_lat = std::numeric_limits<double>::infinity();
    for (const auto& corridor : unique_lane_corridors_) {
      FrenetPosition f(pt, corridor->GetCenterLine());
      if (std::abs(f.lat) < min_lat) {
        min_lat = std::abs(f.lat);
        lc = corridor;
      }
    }
  }
  return lc;
}

}  // namespace map
}  // namespace world
}  // namespace bark
