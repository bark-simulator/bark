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

namespace world {
namespace map {

using modules::opendrive::XodrRoadPtr;
using modules::opendrive::XodrRoad;
using modules::opendrive::XodrLanes;
using modules::opendrive::XodrLane;

// ONLY STORE STUFF
struct Road : public XodrRoad {
  explicit Road(const XodrRoadPtr& road) : XodrRoad(road) {}

  Lanes GetLanes() const {
    return bark_lanes_;
  }

  LanePtr GetLane(unisgned int lane_id) const {
    return bark_lanes_.at(lane_id);
  }

  LanePtr GetNextRoad() const {
    return next_road_;
  }

  RoadPtr next_road_;
  Lanes bark_lanes_;
};

using RoadPtr = std::shared_ptr<Road>;
using Roads = std::map<unsigned int, RoadPtr>;


}  // namespace map
}  // namespace world

#endif  // MODULES_WORLD_MAP_ROAD_HPP_
