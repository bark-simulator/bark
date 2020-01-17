// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_WORLD_MAP_ROAD_CORRIDOR_HPP_
#define MODULES_WORLD_MAP_ROAD_CORRIDOR_HPP_

#include <map>
#include <vector>
#include <string>
#include <boost/functional/hash.hpp>
#include "modules/world/opendrive/opendrive.hpp"
#include "modules/world/map/road.hpp"
#include "modules/world/map/lane.hpp"
#include "modules/world/map/lane_corridor.hpp"
#include "modules/geometry/geometry.hpp"


namespace modules {
namespace world {
namespace map {

using modules::geometry::Line;
using modules::geometry::Polygon;
using modules::world::opendrive::XodrRoadId;


struct RoadCorridor {
  //! Getter
  RoadPtr GetRoad(RoadId road_id) const {
    return roads_.at(road_id);
  }
  Roads GetRoads() const {return roads_;}
  Lanes GetLanes(RoadId road_id) const {
    return this->GetRoad(road_id)->GetLanes();
  }
  LaneCorridorPtr GetLaneCorridor(const LaneId& lane_id) {
    return lane_corridors_.at(lane_id);
  }
  static std::size_t GetHash(
    const std::vector<XodrRoadId>& road_ids) {
    // calculate hash using road_ids
    return boost::hash_range(
      road_ids.begin(),
      road_ids.end());
  }

  //! Setter
  void SetRoads(const Roads& roads) {
    roads_ = roads;
  }
  void SetLaneCorridor(const LaneId& lane_id,
    const LaneCorridorPtr& corr) {
    lane_corridors_[lane_id] = corr;
  }

  Roads roads_;
  std::map<LaneId, LaneCorridorPtr> lane_corridors_;
};
using RoadCorridorPtr = std::shared_ptr<RoadCorridor>;


}  // namespace map
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_MAP_ROAD_CORRIDOR_HPP_
