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

using modules::opendrive::RoadPtr;
using modules::opendrive::Road;
using modules::opendrive::Lanes;
using modules::opendrive::Lane;

// ONLY STORE STUFF
struct BarkRoad : public Road {
  explicit BarkRoad(const RoadPtr& road) : Road(road) {}

  BarkLanes GetLanes() const {
    return bark_lanes_;
  }

  BarkLanePtr GetLane(unisgned int lane_id) const {
    return bark_lanes_.at(lane_id);
  }

  BarkLanePtr GetNextRoad() const {
    return next_road_;
  }

  BarkRoadPtr next_road_;
  BarkLanes bark_lanes_;
};

using BarkRoadPtr = std::shared_ptr<BarkRoad>;
using BarkRoads = std::map<unsigned int, BarkRoadPtr>;


}  // namespace map
}  // namespace world

#endif  // MODULES_WORLD_MAP_ROAD_HPP_
