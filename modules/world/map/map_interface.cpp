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
  bounding_box_ = open_drive_map_->bounding_box();
  return true;
}


bool modules::world::map::MapInterface::FindNearestLanes(
  const Point2d& point,
  const unsigned& num_lanes,
  std::vector<LanePtr>& lanes, 
  bool type_driving_only) const {
  if (!open_drive_map_) {
    return false;
  }
  if (rtree_lane_.empty()) {
    return false;
  }

  std::vector<rtree_lane_value> results_n;

  if (type_driving_only) {
    rtree_lane_.query(boost::geometry::index::nearest(point, num_lanes) && boost::geometry::index::satisfies(is_lane_type),
              std::back_inserter(results_n));
  }
  else {
    rtree_lane_.query(boost::geometry::index::nearest(point, num_lanes), std::back_inserter(results_n));
  }

  if (results_n.empty()) {
    return false;
  }

  lanes.clear();
  for (auto &result : results_n) {
    lanes.push_back(result.second);
  }

  return true;
}

bool modules::world::map::MapInterface::isInLane(const modules::geometry::Point2d& point, LaneId id) const {
  
  std::pair<vertex_t, bool> v = roadgraph_->get_vertex_by_lane_id(id);
  if (v.second) {
    auto polygon = roadgraph_->get_lane_graph()[v.first].polygon;
    if (!polygon) {
      // found vertex has no polygon
      return false;
    }
    else {
      // found vertex has a polygon
      bool point_in_polygon = modules::geometry::Collide(*polygon, point);
      if (point_in_polygon) {
        return true;
      }
      else {
        return false;
      }
    }
  }
  else {
    // no vertex found
    return false;
  }

}

// TODO Klemens: change to LanePtr to Line
std::pair< std::vector<LanePtr>, std::vector<LanePtr> > modules::world::map::MapInterface::ComputeLaneBoundariesHorizon(
      const LaneId& startid, const LaneId& goalid) const {
  std::vector<LaneId> horizon = roadgraph_->find_path(startid, goalid);
  return roadgraph_->ComputeRouteBoundaries(horizon);
}

bool modules::world::map::MapInterface::CalculateDrivingCorridor(const LaneId& startid, const LaneId& goalid,
                                                             Line& inner_line, Line& outer_line, Line& center_line)  const{
  std::pair< std::vector<LanePtr>, std::vector<LanePtr> > route =
      ComputeLaneBoundariesHorizon(startid, goalid);

    if(route.first.empty() || route.second.empty()) {
      return false;
    }

    if (route.first[0]) {
      inner_line = route.first[0]->get_line();
      // inner lane
      for (uint i = 1; i < route.first.size(); i++) {
        if (route.first[i] != NULL) {
          inner_line.ConcatenateLinestring(route.first[i]->get_line());
        }
      }
    }
    if (route.second[0]) {
      outer_line = route.second[0]->get_line();
      // inner lane
      for (uint i = 1; i < route.second.size(); i++) {
        if (route.second[i] != NULL) {
          outer_line.ConcatenateLinestring(route.second[i]->get_line());
        }
      }
    }

    // TODO(@hart): implement working function
    if (route.first[0] != NULL && route.second[0] != NULL) {
      center_line = geometry::ComputeCenterLine(inner_line, outer_line);
      return true;
    } else {
      return false;
    }
}


}  // namespace map
}  // namespace world
}  // namespace modules