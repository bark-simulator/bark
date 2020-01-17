// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_WORLD_MAP_ROAD_HPP_
#define MODULES_WORLD_MAP_ROAD_HPP_

#include <memory>
#include <map>

#include "modules/world/opendrive/road.hpp"
#include "modules/world/map/lane.hpp"

namespace modules {
namespace world {
namespace map {

using RoadId = unsigned int;
using modules::world::opendrive::XodrRoadPtr;
using modules::world::opendrive::XodrRoad;
using modules::world::opendrive::XodrLanes;
using modules::world::opendrive::XodrLane;


struct Road : public XodrRoad {
  explicit Road(const XodrRoadPtr& road) :
    XodrRoad(road),
    next_road_(nullptr) {}

  //! Getter
  Lanes GetLanes() const {
    return lanes_;
  }
  LanePtr GetLane(const LaneId& lane_id) const {
    if (lanes_.count(lane_id) == 0)
      return nullptr;
    return lanes_.at(lane_id);
  }
  std::shared_ptr<Road> GetNextRoad() const {
    return next_road_;
  }

  //! Setter
  void SetLanes(const Lanes& lanes) {
    lanes_ = lanes;
  }
  void SetNextRoad(const std::shared_ptr<Road>& road) {
    next_road_ = road;
  }

  std::shared_ptr<Road> next_road_;
  Lanes lanes_;
};

using RoadPtr = std::shared_ptr<Road>;
using Roads = std::map<RoadId, RoadPtr>;


}  // namespace map
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_MAP_ROAD_HPP_
