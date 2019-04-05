// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_WORLD_MAP_MAP_INTERFACE_HPP_
#define MODULES_WORLD_MAP_MAP_INTERFACE_HPP_

#include <vector>
#include <string>
#include <utility> 
#include <boost/geometry/index/rtree.hpp>

#include "modules/geometry/geometry.hpp"
#include "modules/world/map/map_interface.hpp"
#include "modules/world/map/roadgraph.hpp"

namespace modules {
namespace world {
namespace map {

using modules::world::opendrive::LanePtr;
using modules::world::opendrive::LaneId;
using modules::world::opendrive::OpenDriveMapPtr;
using modules::geometry::Point2d;

using rtree_lane_model = boost::geometry::model::segment<Point2d>;
using rtree_lane_id = LanePtr;
using rtree_lane_value = std::pair<rtree_lane_model, rtree_lane_id>;
using rtree_lane = boost::geometry::index::rtree<rtree_lane_value,
                   boost::geometry::index::linear<16, 4> >;

class MapInterface {
 public:
  bool interface_from_opendrive(const OpenDriveMapPtr& open_drive_map);

  bool get_nearest_lanes(const modules::geometry::Point2d& point,
                         const unsigned& num_lanes,
                         std::vector<opendrive::LanePtr>& lanes);

  std::pair< std::vector<LanePtr>, std::vector<LanePtr> > get_lane_boundary_horizon(const LaneId& startid, const LaneId& goalid);

  std::pair< std::vector<LanePtr>, std::vector<LanePtr> > get_lane_boundaries_from_path(const std::vector<LaneId>& horizon);

  bool set_open_drive_map(OpenDriveMapPtr map) {
    open_drive_map_ = map;
    interface_from_opendrive(open_drive_map_);
    return true;
  }

  bool set_roadgraph(RoadgraphPtr roadgraph) {
    roadgraph_ = roadgraph;
    return true;
  }

  OpenDriveMapPtr get_open_drive_map() { return open_drive_map_; }
  RoadgraphPtr get_roadgraph() { return roadgraph_; }

 private:
  OpenDriveMapPtr open_drive_map_;
  RoadgraphPtr roadgraph_;
  rtree_lane rtree_lane_;
};

using MapInterfacePtr = std::shared_ptr<MapInterface>;


}  // namespace map
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_MAP_MAP_INTERFACE_HPP_