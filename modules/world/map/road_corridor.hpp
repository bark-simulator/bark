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
#include "modules/geometry/geometry.hpp"



namespace modules {
namespace world {
namespace map {

using modules::geometry::Line;
using modules::geometry::Polygon;
using modules::world::opendrive::XodrRoadId;


struct LaneCorridor {
  //! Getter
  LanePtr& GetLeftLane(const LaneId& lane_id) {
    // TODO(@hart): to be implemented
  }
  LanePtr& GetRightLane(const LaneId& lane_id) {
    // TODO(@hart): to be implemented
  }
  Line& GetLeftBoundary() {
    return left_boundary_;
  }
  Line& GetRightBoundary() {
    return right_boundary_;
  }
  Line& GetCenterLine() {
    return center_line_;
  }
  Polygon& GetMergedPolygon() {
    return merged_polygon_;
  }
  std::map<float, LanePtr>& GetLanes() {
    return lanes_;
  }

  //! Setter
  void SetLeftBoundary(const Line& boundary) {
    left_boundary_ = boundary;
  }
  void SetRightBoundary(const Line& boundary) {
    right_boundary_ = boundary;
  }
  void SetCenterLine(const Line& line) {
    center_line_ = line;
  }
  void SetMergedPolygon(const Polygon& poly) {
    merged_polygon_ = poly;
  }
  void SetLane(float s_start, const LanePtr& lane) {
    lanes_[s_start] = lane;
  }

  std::map<float, LanePtr> lanes_;  // s_end, LanePtr
  Line center_line_;
  Polygon merged_polygon_;
  Line left_boundary_;
  Line right_boundary_;
};
using LaneCorridorPtr = std::shared_ptr<LaneCorridor>;


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
    // TODO(@hart): return LaneCorridor
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

  Roads roads_;
  std::vector<LaneCorridorPtr> lane_corridors_;
};
using RoadCorridorPtr = std::shared_ptr<RoadCorridor>;


}  // namespace map
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_MAP_ROAD_CORRIDOR_HPP_
