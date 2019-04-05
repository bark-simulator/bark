// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include "modules/world/map/map_interface.hpp"


namespace modules {
namespace world {
namespace map {

using LaneSegment = boost::geometry::model::segment<Point2d>;
bool modules::world::map::MapInterface::interface_from_opendrive(
  const OpenDriveMapPtr& open_drive_map) {
  open_drive_map_ = open_drive_map;
  rtree_lane_.clear();
  for (auto &road : open_drive_map_->get_roads()) {
    for (auto &lane_section : road.second->get_lane_sections()) {
      for (auto &lane : lane_section->get_lanes())  {
        // TODO(@fortiss): do not use left line and do not want pos 0
        if (lane.second->get_lane_position() != 0){
          auto lane_left_linestring = lane.second->get_line().obj_;
          LaneSegment left_lane_segment(
                              *lane_left_linestring.begin(),
                              *(lane_left_linestring.end()-1));
          rtree_lane_.insert(std::make_pair(left_lane_segment, lane.second));
        }
      }
    }
  }
  return true;
}

bool modules::world::map::MapInterface::get_nearest_lanes(
  const Point2d& point,
  const unsigned& num_lanes,
  std::vector<LanePtr>& lanes) {
  if (!open_drive_map_) {
    return false;
  }
  if (rtree_lane_.empty()) {
    return false;
  }

  std::vector<rtree_lane_value> results_n;
  rtree_lane_.query(boost::geometry::index::nearest(point, num_lanes),
              std::back_inserter(results_n));

  if (results_n.empty()) {
    return false;
  }

  lanes.clear();
  for (auto &result : results_n) {
    lanes.push_back(result.second);
  }

  return true;
}

std::pair< std::vector<LanePtr>, std::vector<LanePtr> > modules::world::map::MapInterface::get_lane_boundary_horizon(const LaneId& startid, const LaneId& goalid) {
  std::vector<LanePtr> inner, outer;
  std::vector<LaneId> horizon = roadgraph_->find_path(startid, goalid);
  if (!horizon.empty()) {
    for (auto &h : horizon) {
      std::pair<vertex_t, bool> v = roadgraph_->get_vertex_by_lane_id(h);
      outer.push_back(roadgraph_->get_lane_graph()[v.first].lane);
      
      LaneId innerid = roadgraph_->get_inner_neighbor(h);
      std::pair<vertex_t, bool> v_inner = roadgraph_->get_vertex_by_lane_id(innerid);
      if (v_inner.second) {
        inner.push_back(roadgraph_->get_lane_graph()[v_inner.first].lane);
      } else {
        inner.push_back(NULL);
      }
    }
  }
  return std::make_pair(inner, outer);
}

}  // namespace map
}  // namespace world
}  // namespace modules