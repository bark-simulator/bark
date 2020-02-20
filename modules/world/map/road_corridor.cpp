// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <utility>
#include <memory>
#include "modules/world/map/road_corridor.hpp"

namespace modules {
namespace world {
namespace map {

std::pair<LaneCorridorPtr, LaneCorridorPtr>
RoadCorridor::GetLeftRightLaneCorridor(const Point2d& pt) const {
  LaneCorridorPtr current_lane_corr = GetCurrentLaneCorridor(pt);
  if(!current_lane_corr) {
      return std::make_pair(
    LaneCorridorPtr(nullptr),
        LaneCorridorPtr(nullptr));
  }
  LanePtr current_lane = current_lane_corr->GetCurrentLane(pt);
  std::shared_ptr<Lane> left_lane, right_lane;
  if (current_lane) {
    left_lane = current_lane_corr->GetCurrentLane(pt)->GetLeftLane();
    right_lane = current_lane_corr->GetCurrentLane(pt)->GetRightLane();
  }
  LaneId left_lane_id = 10000000;
  LaneId right_lane_id = 10000000;
  if (left_lane)
    left_lane_id = left_lane->GetId();
  if (right_lane)
    right_lane_id = right_lane->GetId();
  return std::make_pair(
    GetLaneCorridor(left_lane_id),
    GetLaneCorridor(right_lane_id));
}

}  // namespace map
}  // namespace world
}  // namespace modules
