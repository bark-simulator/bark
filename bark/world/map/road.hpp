// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_MAP_ROAD_HPP_
#define BARK_WORLD_MAP_ROAD_HPP_

#include <map>
#include <memory>

#include "bark/world/map/lane.hpp"
#include "bark/world/opendrive/road.hpp"

namespace bark {
namespace world {
namespace map {

using RoadId = unsigned int;
using bark::world::opendrive::XodrLane;
using bark::world::opendrive::XodrLanes;
using bark::world::opendrive::XodrRoad;
using bark::world::opendrive::XodrRoadPtr;

struct Road : public XodrRoad {
  explicit Road(const XodrRoadPtr& road)
      : XodrRoad(road), next_road_(nullptr) {}

  //! Getter
  Lanes GetLanes() const { return lanes_; }
  LanePtr GetLane(const LaneId& lane_id) const {
    if (lanes_.count(lane_id) == 0) return nullptr;
    return lanes_.at(lane_id);
  }
  std::shared_ptr<Road> GetNextRoad() const { return next_road_; }

  //! Setter
  void SetLanes(const Lanes& lanes) { lanes_ = lanes; }
  void SetNextRoad(const std::shared_ptr<Road>& road) { next_road_ = road; }

  std::shared_ptr<Road> next_road_;
  Lanes lanes_;
};

using RoadPtr = std::shared_ptr<Road>;
using Roads = std::map<RoadId, RoadPtr>;

}  // namespace map
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_MAP_ROAD_HPP_
