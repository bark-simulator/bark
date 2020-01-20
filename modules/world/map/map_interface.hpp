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
#include "modules/world/map/driving_corridor.hpp"
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
  
  void ConcatenateLines(const std::vector<XodrLanePtr>& lanes,
                        Line& line_of_corridor,
                        std::vector< std::pair<int, XodrLaneId> >& lane_ids);


 
  /*
  * Finds the ID's of the nearest lanes to point
  * Note that the point doesn't necessarily lie within the lane of the closest point
  * or inside the lane of any of them.
  */
  bool FindNearestXodrLanes(const Point2d& point,
                         const unsigned& num_lanes,
                         std::vector<opendrive::XodrLanePtr>& lanes,
                         bool type_driving_only = true) const;

  XodrLanePtr FindXodrLane(const Point2d& point) const;

  bool HasCorrectDrivingDirection(const Point2d& point, const float orientation) const;

  bool LineSegmentInsideCorridor(const DrivingCorridorPtr corridor, const Point2d& p1, const Point2d& p2) const;

  bool IsInXodrLane(const Point2d& point, XodrLaneId id) const;
  
  //std::pair< std::vector<XodrLanePtr>, std::vector<XodrLanePtr> > ComputeXodrLaneBoundariesHorizon(
  //                                const XodrLaneId& startid, const XodrLaneId& goalid) const;
  
  //! Compute a DrivingCorridor from the start lane to the goal lane. The goal
  //! must be reachable without a lane change.
  DrivingCorridor ComputeDrivingCorridorFromStartToGoal(const XodrLaneId& startid, const XodrLaneId& goalid);

  //! Compute a DrivingCorridor that ends in a lane neighboring the goal lane in
  //! the same driving direction.
  DrivingCorridor ComputeDrivingCorridorParallelToGoal(const XodrLaneId& startid, const XodrLaneId& goalid);

  DrivingCorridor ComputeDrivingCorridorForRange(std::vector<XodrLaneId> lane_ids);

  bool ComputeAllDrivingCorridors();

  std::vector<PathBoundaries> ComputeAllPathBoundaries(const std::vector<XodrLaneId>& lane_ids) const;

  std::pair<XodrLanePtr, bool> get_inner_neighbor(const XodrLaneId lane_id) const;
  std::pair<XodrLanePtr, bool> get_outer_neighbor(const XodrLaneId lane_id) const;
  std::vector<XodrLaneId> get_successor_lanes(const XodrLaneId lane_id) const;

  std::vector<DrivingCorridorPtr> GetAdjacentDrivingCorridorsSameDirection(const DrivingCorridorPtr corridor, const Pose& pose);
  std::vector<DrivingCorridorPtr> GetSplittingDrivingCorridors(const DrivingCorridorPtr corridor, const Pose& pose);

  virtual std::pair<Point2d, Point2d> BoundingBox() const { return bounding_box_;}

  bool set_open_drive_map(OpenDriveMapPtr map) {
    open_drive_map_ = map;
    interface_from_opendrive(open_drive_map_);
    return true;
  }

  bool set_roadgraph(RoadgraphPtr roadgraph) {
    roadgraph_ = roadgraph;
    return true;
  }

  XodrLanePtr get_lane(const XodrLaneId& id) const {
    return roadgraph_->get_laneptr(id);
  }

  std::vector<DrivingCorridorPtr> get_all_corridors() const { return all_corridors_; }

  //! Functions
  OpenDriveMapPtr get_open_drive_map() { return open_drive_map_; }
  RoadgraphPtr get_roadgraph() { return roadgraph_; }


  //! RoadCorridor
  void CalculateLaneCorridors(
    RoadCorridorPtr& road_corridor,
    const RoadPtr& road);
  void CalculateLaneCorridors(RoadCorridorPtr& road_corridor);
  LanePtr GenerateRoadCorridorLane(const XodrLanePtr& lane);
  RoadPtr GenerateRoadCorridorRoad(const XodrRoadId& road_id);
  RoadPtr GetNextRoad(const XodrRoadId& current_road_id,
    const Roads& roads,
    const std::vector<XodrRoadId>& road_ids) const;
  void GenerateRoadCorridor(const std::vector<XodrRoadId>& road_ids,
    const XodrDrivingDirection& driving_direction);
  
  RoadCorridorPtr GetRoadCorridor(std::size_t hash) {
    return road_corridors_.at(hash);
  }
  LaneId FindCurrentLane(const Point2d& pt) {
    return FindXodrLane(pt)->get_id();
  }
  RoadId FindCurrentRoad(const Point2d& pt) {
    XodrRoadId road_id = roadgraph_->GetRoadForLaneId(
      FindCurrentLane(pt));
    return road_id;
  }


 private:
  OpenDriveMapPtr open_drive_map_;
  RoadgraphPtr roadgraph_;
  std::vector<DrivingCorridorPtr> all_corridors_;
  rtree_lane rtree_lane_;
  std::pair<Point2d, Point2d> bounding_box_;
  std::map<std::size_t, RoadCorridorPtr> road_corridors_;

  static bool is_lane_type(rtree_lane_value const &m) {return (m.second->get_lane_type() == XodrLaneType::DRIVING); }

};

using MapInterfacePtr = std::shared_ptr<MapInterface>;


}  // namespace map
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_MAP_MAP_INTERFACE_HPP_