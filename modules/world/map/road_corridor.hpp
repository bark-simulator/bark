// Copyright (c) 2020 Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_WORLD_MAP_ROAD_CORRIDOR_HPP_
#define MODULES_WORLD_MAP_ROAD_CORRIDOR_HPP_

#include <map>
#include <vector>
#include <string>
#include <utility>
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
using modules::geometry::Point2d;
using modules::geometry::Within;
using modules::geometry::BufferPolygon;
using modules::world::opendrive::XodrRoadId;
using modules::world::opendrive::XodrDrivingDirection;


struct RoadCorridor {
  //! Getter
  RoadPtr GetRoad(const RoadId& road_id) const {
    if (roads_.count(road_id) == 0)
      return nullptr;
    return roads_.at(road_id);
  }
  Roads GetRoads() const { return roads_; }
  Polygon GetPolygon() const { return road_polygon_; }
  Lanes GetLanes(const RoadId& road_id) const {
    return this->GetRoad(road_id)->GetLanes();
  }
  LaneCorridorPtr GetLaneCorridor(const LaneId& lane_id) const {
    if (lane_corridors_.count(lane_id) == 0)
      return nullptr;
    return lane_corridors_.at(lane_id);
  }
  std::vector<LaneCorridorPtr> GetUniqueLaneCorridors() const {
    return unique_lane_corridors_;
  }
  std::map<LaneId, LaneCorridorPtr> GetLaneCorridorMap() const {
    return lane_corridors_;
  }
  std::vector<XodrRoadId> GetRoadIds() const {
    return road_ids_;
  }
  LaneCorridorPtr GetCurrentLaneCorridor(const Point2d& pt) const {
    for (const auto& lane_corr : unique_lane_corridors_) {
      if (Collide(pt, lane_corr->GetMergedPolygon()))
        return lane_corr;
    }
    return nullptr;
  }
  std::pair<LaneCorridorPtr, LaneCorridorPtr>
  GetLeftRightLaneCorridor(const Point2d& pt) const;

  static std::size_t GetHash(
    const XodrDrivingDirection& driving_direction,
    const std::vector<XodrRoadId>& road_ids) {
    // calculate hash using driving_direction and road_ids
    std::size_t road_id_hash = boost::hash_range(
      road_ids.begin(),
      road_ids.end());
    boost::hash_combine(road_id_hash, driving_direction);
    return road_id_hash;
  }

  //! Setter
  void SetRoads(const Roads& roads) {
    roads_ = roads;
  }
  void SetLaneCorridor(const LaneId& lane_id,
    const LaneCorridorPtr& corr) {
    lane_corridors_[lane_id] = corr;
    // we only want to push it if it is not included
    if (std::find(unique_lane_corridors_.begin(),
                  unique_lane_corridors_.end(),
                  corr) == unique_lane_corridors_.end()) {
      unique_lane_corridors_.push_back(corr);
    }
  }
  void SetUniqueLaneCorridors(
    const std::vector<LaneCorridorPtr>& unique_lane_corridors) {
    unique_lane_corridors_ = unique_lane_corridors;
  }
  void SetLaneCorridorMap(
    const std::map<LaneId, LaneCorridorPtr>& lane_corridors) {
    lane_corridors_ = lane_corridors;
  }
  bool ComputeRoadPolygon(double buffer_dist = 0.3) {
    Polygon merged_polygon;
    // merge all lane polygons
    for (const auto& lane_corr : unique_lane_corridors_) {
      const auto& lanes = lane_corr->GetLanes();
      for (const auto& lane : lanes) {
        Polygon poly_buffered;
        BufferPolygon(lane.second->GetPolygon(), buffer_dist, &poly_buffered);
        merged_polygon.ConcatenatePolygons(poly_buffered);
      }
    }
    Polygon poly_buffered_merged;
    BufferPolygon(merged_polygon, -buffer_dist, &poly_buffered_merged);
    road_polygon_ = poly_buffered_merged;
    return true;
  }
  void SetPolygon(const Polygon& poly) { road_polygon_ = poly; }
  void SetRoadIds(
    const std::vector<XodrRoadId>& road_ids) {
    road_ids_ = road_ids;
  }

  Roads roads_;
  Polygon road_polygon_;
  std::vector<LaneCorridorPtr> unique_lane_corridors_;
  std::vector<XodrRoadId> road_ids_;
  std::map<LaneId, LaneCorridorPtr> lane_corridors_;
};
using RoadCorridorPtr = std::shared_ptr<RoadCorridor>;


}  // namespace map
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_MAP_ROAD_CORRIDOR_HPP_
