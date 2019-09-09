// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/world/map/map_interface.hpp"

namespace modules
{
namespace world
{
namespace map
{

using LaneSegment = boost::geometry::model::segment<Point2d>;
bool MapInterface::interface_from_opendrive(
    const OpenDriveMapPtr &open_drive_map)
{
  open_drive_map_ = open_drive_map;
  rtree_lane_.clear();
  for (auto &road : open_drive_map_->get_roads())
  {
    for (auto &lane_section : road.second->get_lane_sections())
    {
      for (auto &lane : lane_section->get_lanes())
      {
        // TODO(@fortiss): do not use left line and do not want pos 0
        if (lane.second->get_lane_position() != 0)
        {
          auto lane_left_linestring = lane.second->get_line().obj_;
          LaneSegment left_lane_segment(
              *lane_left_linestring.begin(),
              *(lane_left_linestring.end() - 1));
          rtree_lane_.insert(std::make_pair(left_lane_segment, lane.second));
        }
      }
    }
  }
  bounding_box_ = open_drive_map_->bounding_box();
  return true;
}

void MapInterface::ConcatenateLines(const std::vector<LanePtr> &lanes,
                                    Line &line_of_corridor,
                                    std::vector<std::pair<int, LaneId>> &lane_ids)
{
  if (lanes.size() > 0)
  {
    line_of_corridor = lanes.at(0)->get_line();
    lane_ids.push_back(std::pair<int, LaneId>(0, lanes.at(0)->get_id()));
    for (uint i = 1; i < lanes.size(); i++)
    {
      if (lanes.at(i) != NULL)
      {
        lane_ids.push_back(std::pair<int, LaneId>(line_of_corridor.size(),
                                                  lanes.at(i)->get_id()));
        line_of_corridor.ConcatenateLinestring(lanes.at(i)->get_line());
      }
    }
  }
}

bool MapInterface::FindNearestLanes(const Point2d &point,
                                    const unsigned &num_lanes,
                                    std::vector<LanePtr> &lanes,
                                    bool type_driving_only) const
{
  if (!open_drive_map_)
  {
    return false;
  }
  if (rtree_lane_.empty())
  {
    return false;
  }

  std::vector<rtree_lane_value> results_n;

  if (type_driving_only)
  {
    rtree_lane_.query(boost::geometry::index::nearest(point, num_lanes) && boost::geometry::index::satisfies(is_lane_type),
                      std::back_inserter(results_n));
  }
  else
  {
    rtree_lane_.query(boost::geometry::index::nearest(point, num_lanes), std::back_inserter(results_n));
  }

  if (results_n.empty())
  {
    return false;
  }

  lanes.clear();
  for (auto &result : results_n)
  {
    lanes.push_back(result.second);
  }

  return true;
}

bool MapInterface::isInLane(const modules::geometry::Point2d &point, LaneId id) const
{

  std::pair<vertex_t, bool> v = roadgraph_->get_vertex_by_lane_id(id);
  if (v.second)
  {
    auto polygon = roadgraph_->get_lane_graph()[v.first].polygon;
    if (!polygon)
    {
      // found vertex has no polygon
      return false;
    }
    else
    {
      // found vertex has a polygon
      bool point_in_polygon = modules::geometry::Collide(*polygon, point);
      if (point_in_polygon)
      {
        return true;
      }
      else
      {
        return false;
      }
    }
  }
  else
  {
    // no vertex found
    return false;
  }
}

DrivingCorridor MapInterface::ComputeDrivingCorridorFromStartToGoal(const LaneId &startid, const LaneId &goalid)
{
  std::vector<LaneId> ids = roadgraph_->find_path(startid, goalid);
  return ComputeDrivingCorridorForRange(ids);
}

DrivingCorridor MapInterface::ComputeDrivingCorridorForRange(std::vector<LaneId> lane_ids)
{
  std::pair<std::vector<LanePtr>, std::vector<LanePtr>> route =
      roadgraph_->ComputeRouteBoundaries(lane_ids);

  DrivingCorridor dc;
  std::vector<std::pair<int, LaneId>> dummy;
  ConcatenateLines(route.first, dc.inner, dc.lane_ids_);
  ConcatenateLines(route.second, dc.outer, dummy);
  if (route.first[0] != NULL && route.second[0] != NULL)
  {
    dc.center = ComputeCenterLine(dc.inner, dc.outer);
  }
  dc.computed = true;
  return dc;
}

bool MapInterface::ComputeAllDrivingCorridors()
{

  std::vector<LaneId> ids = roadgraph_->get_all_laneids();
  // get all unique ids that are driving corridors

  auto all_path_boundaries = ComputeAllPathBoundaries(ids);

  for (auto const &path_boundaries : all_path_boundaries)
  {

    std::vector<LanePtr> route_inner, route_outer;
    // get from vector of pairs to pair of vectors
    for (auto path_it = path_boundaries.begin(); path_it != path_boundaries.end(); path_it++)
    {
      route_inner.push_back(path_it->first);
      route_outer.push_back(path_it->second);
    }

    DrivingCorridorPtr dc = std::make_shared<DrivingCorridor>();

    std::vector<std::pair<int, LaneId>> dummy;
    ConcatenateLines(route_inner, dc->inner, dummy);
    ConcatenateLines(route_outer, dc->outer, dc->lane_ids_);
    if (route_inner[0] != NULL && route_outer[0] != NULL)
    {
      dc->center = ComputeCenterLine(dc->inner, dc->outer);
    }
    dc->computed = true;

    all_corridors_.push_back(dc);
  }

  return true;
}

std::vector<PathBoundaries> modules::world::map::MapInterface::ComputeAllPathBoundaries(
    const std::vector<LaneId> &lane_ids) const
{
  std::vector<LaneEdgeType> successor_edges = {LaneEdgeType::SUCCESSOR_EDGE};
  std::vector<std::vector<LaneId>> all_paths = roadgraph_->find_all_paths_in_subgraph(successor_edges, lane_ids);

  std::vector<PathBoundaries> all_path_boundaries;
  for (auto const &path : all_paths)
  {
    std::vector<std::pair<LanePtr, LanePtr>> path_boundaries;
    for (auto const &path_segment : path)
    {
      std::pair<LanePtr, LanePtr> lane_boundaries = roadgraph_->ComputeLaneBoundaries(path_segment);
      path_boundaries.push_back(lane_boundaries);
    }
    all_path_boundaries.push_back(path_boundaries);
  }
  return all_path_boundaries;
}

std::pair<LanePtr, bool> modules::world::map::MapInterface::get_inner_neighbor(const LaneId lane_id) const
{
  std::pair<LaneId, bool> inner_neighbor = roadgraph_->get_inner_neighbor(lane_id);
  if (inner_neighbor.second)
  {
    return std::make_pair(roadgraph_->get_laneptr(inner_neighbor.first), true);
  }
  else
  {
    return std::make_pair(nullptr, false);
  }
}

std::pair<LanePtr, bool> modules::world::map::MapInterface::get_outer_neighbor(const LaneId lane_id) const
{
  std::pair<LaneId, bool> outer_neighbor = roadgraph_->get_outer_neighbor(lane_id);
  if (outer_neighbor.second)
  {
    return std::make_pair(roadgraph_->get_laneptr(outer_neighbor.first), true);
  }
  else
  {
    return std::make_pair(nullptr, false);
  }
}

std::vector<LaneId> modules::world::map::MapInterface::get_successor_lanes(const LaneId lane_id) const
{
  return roadgraph_->get_successor_lanes(lane_id);
}

std::vector<DrivingCorridorPtr> MapInterface::GetAdjacentDrivingCorridors(const DrivingCorridorPtr corridor, const Pose& pose)
{
  
  uint idx = FindNearestIdx(corridor->get_center(), Point2d(pose[0], pose[1]));

  std::vector<DrivingCorridorPtr> adj_corridors;

  for (auto &lane_id : corridor->get_lane_ids())
  {
    if (lane_id.first >= idx)
      {
        std::vector<std::pair<LanePtr, bool>> lane_neighbors;
        lane_neighbors.push_back(get_inner_neighbor(lane_id.second));
        lane_neighbors.push_back(get_outer_neighbor(lane_id.second));

        for (auto &lane_neighbor : lane_neighbors) {
          if (lane_neighbor.second && lane_neighbor.first->get_lane_position() != 0)
          {
            for (auto &corridor_rhs : all_corridors_)
            { // finding corridors that have lane lane_neighbor as member
              for (auto &lane_id_rhs : corridor_rhs->get_lane_ids())
              { 
                if (roadgraph_->get_laneptr(lane_id.second)->get_lane_position() * lane_neighbor.first->get_lane_position() < 0) {
                  // sign of lane positions is different --> definition of different driving directions
                  break;
                }
                if (lane_id_rhs.second == lane_neighbor.first->get_id())
                { // if lane from driving_corridor is equal to lane_neighbor
                  if ((std::find(adj_corridors.begin(), adj_corridors.end(), corridor_rhs) == adj_corridors.end()))
                  { // right_adj_corridors does not contain corridor_rhs
                    adj_corridors.push_back(corridor_rhs);
                }
              }
            }
          }
        }
      }
    }
  }
  return adj_corridors;
  
}

} // namespace map
} // namespace world
} // namespace modules
