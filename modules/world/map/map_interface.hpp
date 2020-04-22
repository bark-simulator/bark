// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_WORLD_MAP_MAP_INTERFACE_HPP_
#define MODULES_WORLD_MAP_MAP_INTERFACE_HPP_

#include <vector>
#include <map>
#include <string>
#include <utility> 
#include <boost/geometry/index/rtree.hpp>
#include "modules/geometry/geometry.hpp"
#include "modules/world/map/map_interface.hpp"
#include "modules/world/map/roadgraph.hpp"
#include "modules/world/map/road_corridor.hpp"


namespace modules {
namespace world {
namespace map {

using modules::world::opendrive::XodrLanePtr;
using modules::world::opendrive::XodrLaneId;
using modules::world::opendrive::XodrDrivingDirection;
using modules::world::opendrive::OpenDriveMapPtr;
using modules::geometry::Point2d;
using modules::geometry::Line;

using rtree_lane_model = boost::geometry::model::segment<Point2d>;
using rtree_lane_id = XodrLanePtr;
using rtree_lane_value = std::pair<rtree_lane_model, rtree_lane_id>;
using rtree_lane = boost::geometry::index::rtree<rtree_lane_value,
                   boost::geometry::index::linear<16, 4> >;
using PathBoundaries = std::vector<std::pair<XodrLanePtr, XodrLanePtr>>;

class MapInterface {
 public:
  bool interface_from_opendrive(const OpenDriveMapPtr& open_drive_map);

  bool FindNearestXodrLanes(const Point2d& point,
                            const unsigned& num_lanes,
                            std::vector<opendrive::XodrLanePtr>& lanes,
                            bool type_driving_only = true) const;

  XodrLanePtr FindXodrLane(const Point2d& point) const;

  bool IsInXodrLane(const Point2d& point, XodrLaneId id) const;
  
  std::vector<PathBoundaries>
  ComputeAllPathBoundaries(const std::vector<XodrLaneId>& lane_ids) const;
  std::pair<XodrLanePtr, bool>
  GetInnerNeighbor(const XodrLaneId lane_id) const;
  std::pair<XodrLanePtr, bool>
  GetOuterNeighbor(const XodrLaneId lane_id) const;
  std::vector<XodrLaneId> GetSuccessorLanes(const XodrLaneId lane_id) const;

  virtual std::pair<Point2d, Point2d> BoundingBox() const {
    return bounding_box_;
  }

  bool SetOpenDriveMap(OpenDriveMapPtr map) {
    open_drive_map_ = map;
    interface_from_opendrive(open_drive_map_);
    return true;
  }

  bool SetRoadgraph(RoadgraphPtr roadgraph) {
    roadgraph_ = roadgraph;
    return true;
  }

  XodrLanePtr GetLane(const XodrLaneId& id) const {
    return roadgraph_->GetLanePtr(id);
  }

  //! Functions
  OpenDriveMapPtr GetOpenDriveMap() { return open_drive_map_; }
  RoadgraphPtr GetRoadgraph() { return roadgraph_; }

  //! RoadCorridor
  void CalculateLaneCorridors(
    RoadCorridorPtr& road_corridor,
    const RoadPtr& road);
  void CalculateLaneCorridors(RoadCorridorPtr& road_corridor, const XodrRoadId&);
  LanePtr GenerateRoadCorridorLane(const XodrLanePtr& lane);
  RoadPtr GenerateRoadCorridorRoad(const XodrRoadId& road_id);
  RoadPtr GetNextRoad(const XodrRoadId& current_road_id,
    const Roads& roads,
    const std::vector<XodrRoadId>& road_ids) const;
  void GenerateRoadCorridor(const std::vector<XodrRoadId>& road_ids,
    const XodrDrivingDirection& driving_direction);
  RoadCorridorPtr GenerateRoadCorridor(const XodrRoadId& start_road_id,
    const XodrRoadId& end_road_id);
  RoadCorridorPtr GenerateRoadCorridor(
    const modules::geometry::Point2d& start_point,
    const modules::geometry::Polygon& goal_region);
  bool XodrLaneIdAtPolygon(
    const modules::geometry::Polygon&  polygon,
    XodrLaneId& found_lane_id) const;
  RoadCorridorPtr GetRoadCorridor(const std::vector<XodrRoadId>& road_ids,
    const XodrDrivingDirection& driving_direction) {
      std::size_t rc_hash = RoadCorridor::GetHash(driving_direction, road_ids);
    if (road_corridors_.count(rc_hash) == 0)
      return nullptr;
    return road_corridors_.at(rc_hash);
  }

  LaneId FindCurrentLane(const Point2d& pt) {
    return FindXodrLane(pt)->GetId();
  }
  RoadId FindCurrentRoad(const Point2d& pt) {
    XodrRoadId road_id = roadgraph_->GetRoadForLaneId(
      FindCurrentLane(pt));
    return road_id;
  }


 private:
  OpenDriveMapPtr open_drive_map_;
  RoadgraphPtr roadgraph_;
  rtree_lane rtree_lane_;
  std::pair<Point2d, Point2d> bounding_box_;
  std::map<std::size_t, RoadCorridorPtr> road_corridors_;

  static bool IsLaneType(rtree_lane_value const &m) {
    return (m.second->GetLaneType() == XodrLaneType::DRIVING);
  }
};

using MapInterfacePtr = std::shared_ptr<MapInterface>;


}  // namespace map
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_MAP_MAP_INTERFACE_HPP_
