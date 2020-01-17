// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include <math.h>
#include <random>
#include "modules/world/map/map_interface.hpp"

namespace modules {
namespace world {
namespace map {

using LineSegment = boost::geometry::model::segment<Point2d>;
bool MapInterface::interface_from_opendrive(
    const OpenDriveMapPtr &open_drive_map) {
  open_drive_map_ = open_drive_map;

  RoadgraphPtr roadgraph(new Roadgraph());
  roadgraph->Generate(open_drive_map);
  roadgraph_ = roadgraph;

  rtree_lane_.clear();
  for (auto &road : open_drive_map_->get_roads()) {
    for (auto &lane_section : road.second->get_lane_sections()) {
      for (auto &lane : lane_section->get_lanes()) {
        if (lane.second->get_lane_position() != 0) {
          std::pair<XodrLanePtr, XodrLanePtr> lb =
            roadgraph_->ComputeXodrLaneBoundaries(lane.second->get_id());
          if (lb.first && lb.second) {
            auto center = ComputeCenterLine(lb.first->get_line(), lb.second->get_line());
            LineSegment center_lane_segment(*center.begin(),*(center.end() - 1));
            rtree_lane_.insert(std::make_pair(center_lane_segment, lane.second));
          }
        }
      }
    }
  }

  bounding_box_ = open_drive_map_->bounding_box();
  return true;
}

void MapInterface::ConcatenateLines(const std::vector<XodrLanePtr> &lanes,
                                    Line &line_of_corridor,
                                    std::vector<std::pair<int, XodrLaneId>> &lane_ids) {
  if (lanes.size() > 0) {
    line_of_corridor = lanes.at(0)->get_line();
    lane_ids.push_back(std::pair<int, XodrLaneId>(0, lanes.at(0)->get_id()));
    for (uint i = 1; i < lanes.size(); i++)
    {
      if (lanes.at(i) != NULL)
      {
        lane_ids.push_back(std::pair<int, XodrLaneId>(line_of_corridor.size(),
                                                  lanes.at(i)->get_id()));
        line_of_corridor.ConcatenateLinestring(lanes.at(i)->get_line());
      }
    }
  }
}

bool MapInterface::FindNearestXodrLanes(const Point2d &point,
                                    const unsigned &num_lanes,
                                    std::vector<XodrLanePtr> &lanes,
                                    bool type_driving_only) const
{
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

XodrLanePtr MapInterface::FindXodrLane(const Point2d& point) const {
  
  XodrLanePtr lane;
  std::vector<XodrLanePtr> nearest_lanes;
  
  // TODO(@esterle): parameter (20) auslagern
  if(!FindNearestXodrLanes(point, 20, nearest_lanes, false)) {
    return nullptr;
  }
  
  for (auto &close_lane : nearest_lanes) {
    if(IsInXodrLane(point, close_lane->get_id())) {
      lane = close_lane;
      return lane;
    }
  }
  
  return nullptr;
}

bool MapInterface::HasCorrectDrivingDirection(const Point2d& point, const float orientation) const {

  XodrLanePtr lane = FindXodrLane(point);
  if (!lane) {
    return false;
  }

  float orientation_normalized = remainder(orientation, 2*M_PI);  // normalize to [0, 2pi]

  float s = get_nearest_s(lane->get_line(), point);
  float orientation_at_s = get_tangent_angle_at_s(lane->get_line(), s);
  orientation_at_s = remainder(orientation_at_s, 2*M_PI);  // normalize to [0, 2pi]

  bool same_orientation_as_road = (std::fabs(orientation_at_s - orientation_normalized) <= M_PI_2);

  bool point_is_in_right_lane = lane->get_lane_position() < 0;  // open drive definition

  if (point_is_in_right_lane) {
    if (same_orientation_as_road) {
      return true;
    }
    else {
      return false;
    }
  }
  else {
    if (same_orientation_as_road) {
      return false;
    }
    else {
      return true;
    }
  }
}

bool MapInterface::LineSegmentInsideCorridor(const DrivingCorridorPtr corridor, const Point2d& p1, const Point2d& p2) const {
  
  Polygon polygon = corridor->CorridorPolygon();
  
  Line line;
  line.add_point(p1);
  line.add_point(p2);

  bool line_in_polygon = modules::geometry::Within(line, polygon);
  if (line_in_polygon)
  {
    return true;
  }
  else
  {
    return false;
  }
}
  
bool MapInterface::IsInXodrLane(const Point2d &point, XodrLaneId id) const
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

DrivingCorridor MapInterface::ComputeDrivingCorridorFromStartToGoal(
  const XodrLaneId &startid, const XodrLaneId &goalid)
{
  std::vector<XodrLaneId> ids = roadgraph_->find_path(startid, goalid);
  if (!ids.empty()) {
    return ComputeDrivingCorridorForRange(ids);
  } else {
    return DrivingCorridor();
  }
}

DrivingCorridor MapInterface::ComputeDrivingCorridorParallelToGoal(
  const XodrLaneId& startid, const XodrLaneId& goalid)
{
  std::vector<XodrLaneId> goal_neighbors = roadgraph_->get_all_neighbors(goalid);
  for (auto const &goal_neighbor : goal_neighbors) {
    std::vector<XodrLaneId> ids = roadgraph_->find_path(startid, goal_neighbor);
    if (ids.size() > 0) {
      // Found a target lane that is parallel to the goal lane
      return ComputeDrivingCorridorForRange(ids);
    }
  }

  // No lane parallel to the goal was found
  return DrivingCorridor();
}

DrivingCorridor MapInterface::ComputeDrivingCorridorForRange(std::vector<XodrLaneId> lane_ids)
{
  std::pair<std::vector<XodrLanePtr>, std::vector<XodrLanePtr>> route =
      roadgraph_->ComputeRouteBoundaries(lane_ids);

  DrivingCorridor dc;
  std::vector<std::pair<int, XodrLaneId>> dummy;
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

  std::vector<XodrLaneId> ids = roadgraph_->get_all_laneids();
  // get all unique ids that are driving corridors

  auto all_path_boundaries = ComputeAllPathBoundaries(ids);

  for (auto const &path_boundaries : all_path_boundaries)
  {

    std::vector<XodrLanePtr> route_inner, route_outer;
    // get from vector of pairs to pair of vectors
    for (auto path_it = path_boundaries.begin(); path_it != path_boundaries.end(); path_it++)
    {
      route_inner.push_back(path_it->first);
      route_outer.push_back(path_it->second);
    }

    DrivingCorridorPtr dc = std::make_shared<DrivingCorridor>();

    std::vector<std::pair<int, XodrLaneId>> dummy;
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
    const std::vector<XodrLaneId> &lane_ids) const
{
  std::vector<XodrLaneEdgeType> successor_edges = {XodrLaneEdgeType::SUCCESSOR_EDGE};
  std::vector<std::vector<XodrLaneId>> all_paths = roadgraph_->find_all_paths_in_subgraph(successor_edges, lane_ids);

  std::vector<PathBoundaries> all_path_boundaries;
  for (auto const &path : all_paths)
  {
    std::vector<std::pair<XodrLanePtr, XodrLanePtr>> path_boundaries;
    for (auto const &path_segment : path)
    {
      std::pair<XodrLanePtr, XodrLanePtr> lane_boundaries = roadgraph_->ComputeXodrLaneBoundaries(path_segment);
      path_boundaries.push_back(lane_boundaries);
    }
    all_path_boundaries.push_back(path_boundaries);
  }
  return all_path_boundaries;
}

std::pair<XodrLanePtr, bool> modules::world::map::MapInterface::get_inner_neighbor(const XodrLaneId lane_id) const
{
  std::pair<XodrLaneId, bool> inner_neighbor = roadgraph_->get_inner_neighbor(lane_id);
  if (inner_neighbor.second)
  {
    return std::make_pair(roadgraph_->get_laneptr(inner_neighbor.first), true);
  }
  else
  {
    return std::make_pair(nullptr, false);
  }
}

std::pair<XodrLanePtr, bool> modules::world::map::MapInterface::get_outer_neighbor(const XodrLaneId lane_id) const
{
  std::pair<XodrLaneId, bool> outer_neighbor = roadgraph_->get_outer_neighbor(lane_id);
  if (outer_neighbor.second)
  {
    return std::make_pair(roadgraph_->get_laneptr(outer_neighbor.first), true);
  }
  else
  {
    return std::make_pair(nullptr, false);
  }
}

std::vector<XodrLaneId> modules::world::map::MapInterface::get_successor_lanes(const XodrLaneId lane_id) const
{
  return roadgraph_->get_successor_lanes(lane_id);
}

std::vector<DrivingCorridorPtr> MapInterface::GetAdjacentDrivingCorridorsSameDirection(const DrivingCorridorPtr corridor, const Pose &pose)
{

  uint idx = FindNearestIdx(corridor->get_center(), Point2d(pose[0], pose[1]));

  std::vector<DrivingCorridorPtr> adj_corridors;

  for (auto &lane_id : corridor->get_lane_ids())
  {
    if (lane_id.first >= idx)
    {
      std::vector<std::pair<XodrLanePtr, bool>> lane_neighbors;
      std::pair<XodrLanePtr, bool> inner_lane, outer_lane;

      inner_lane = get_inner_neighbor(lane_id.second);
      lane_neighbors.push_back(inner_lane);
      if (inner_lane.second && inner_lane.first->get_lane_position() == 0)
      { // if neigbouring lane is plan view, look again
        lane_neighbors.push_back(get_inner_neighbor(inner_lane.first->get_id()));
      }

      outer_lane = get_outer_neighbor(lane_id.second);
      lane_neighbors.push_back(outer_lane);
      if (outer_lane.second && outer_lane.first->get_lane_position() == 0)
      { // if neigbouring lane is plan view, look again
        lane_neighbors.push_back(get_outer_neighbor(outer_lane.first->get_id()));
      }

      for (auto &lane_neighbor : lane_neighbors)
      {
        if (lane_neighbor.second && lane_neighbor.first->get_lane_position() != 0)
        {
          for (auto &corridor_rhs : all_corridors_)
          { // finding corridors that have lane lane_neighbor as member
            for (auto &lane_id_rhs : corridor_rhs->get_lane_ids())
            {
              if (roadgraph_->get_laneptr(lane_id.second)->get_lane_position() * lane_neighbor.first->get_lane_position() < 0)
              {
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

std::vector<DrivingCorridorPtr> MapInterface::GetSplittingDrivingCorridors(const DrivingCorridorPtr corridor, const Pose &pose)
{

  uint idx = FindNearestIdx(corridor->get_center(), Point2d(pose[0], pose[1]));

  std::vector<DrivingCorridorPtr> splitting_corridors;
  size_t idx_current_lane_id;
  bool valid_pose = false;
  for (size_t i = 0; i < corridor->get_lane_ids().size() - 1; ++i)
  {
    if (corridor->get_lane_ids().at(i).first >= idx)
    {
      idx_current_lane_id = i;
      valid_pose = true;
      //std::cout << "corridor->get_lane_ids().at(idx_current_lane_id).second " << corridor->get_lane_ids().at(idx_current_lane_id).second << std::endl;
      break;
    }
  }
  if (valid_pose)
  {
    for (auto &corridor_rhs : all_corridors_)
    { // finding corridors that have lane lane_neighbor as member
      //std::cout << "corridor_rhs " << std::endl;

      //for (auto &l : corridor_rhs->get_lane_ids())
      //{
      //  std::cout << " " << l.second << std::endl;
      //}
      for (size_t i = 0; i < corridor_rhs->get_lane_ids().size(); ++i)
      {
        if (corridor_rhs->get_lane_ids().at(i).second == corridor->get_lane_ids().at(idx_current_lane_id).second)
        { // if lane from driving_corridor is equal to lane_neighbor

          std::vector<XodrLaneId> corridor_ids;
          //std::cout << "matching corridor " << std::endl;
          for (auto &l : corridor->get_lane_ids())
          {
            //std::cout << " " << l.second << std::endl;
            corridor_ids.push_back(l.second);
          }
          std::vector<XodrLaneId> corridor_rhs_ids;
          //std::cout << "matching corridor_rhs " << std::endl;
          for (auto &l : corridor_rhs->get_lane_ids())
          {
            //std::cout << " " << l.second << std::endl;
            corridor_rhs_ids.push_back(l.second);
          }
          bool result = std::equal(corridor_ids.begin() + idx_current_lane_id,
                                   corridor_ids.end(),
                                   corridor_rhs_ids.begin() + i);
          if (!result)
          { // if they were equal, there would be no splitting
            if ((std::find(splitting_corridors.begin(), splitting_corridors.end(), corridor_rhs) == splitting_corridors.end()))
            { // right_adj_corridors does not contain corridor_rhs
              splitting_corridors.push_back(corridor_rhs);
            }
          }
        }
      }
    }
  }
  return splitting_corridors;
}

// new functionalities
void MapInterface::CalculateLaneCorridors(
  RoadCorridorPtr& road_corridor) {
  RoadPtr first_road = road_corridor->GetRoads()[0];
  Lanes lanes = first_road->GetLanes();
  for (auto& lane : lanes) {
    LaneCorridorPtr lane_corridor = std::make_shared<LaneCorridor>();
    LanePtr current_lane = lane.second;
    float total_s = current_lane->GetCenterLine().length();
    lane_corridor->SetCenterLine(current_lane->GetCenterLine());
    lane_corridor->SetMergedPolygon(current_lane->GetPolygon());
    // TODO(@hart): is this correct?
    lane_corridor->SetLeftBoundary(
      current_lane->GetLeftBoundary().line_);
    lane_corridor->SetRightBoundary(
      current_lane->GetRightBoundary().line_);
    lane_corridor->SetLane(
      total_s,
      current_lane);
    // add initial lane
    road_corridor->SetLaneCorridor(current_lane->get_id(), lane_corridor);

    LanePtr next_lane = current_lane;
    for (;;) {
      next_lane = next_lane->GetNextLane();
      if (next_lane == nullptr)
        break;
      lane_corridor->GetCenterLine().ConcatenateLinestring(
        next_lane->GetCenterLine());
      lane_corridor->GetLeftBoundary().ConcatenateLinestring(
        next_lane->GetLeftBoundary().line_);
      lane_corridor->GetRightBoundary().ConcatenateLinestring(
        next_lane->GetRightBoundary().line_);
      lane_corridor->GetMergedPolygon().ConcatenatePolygons(
        next_lane->GetPolygon());

      total_s = lane_corridor->GetCenterLine().length();
      lane_corridor->SetLane(
        total_s,
        next_lane);
      // all following lanes should point to the same LaneCorridor
      road_corridor->SetLaneCorridor(next_lane->get_id(), lane_corridor);
    }
  }
}

LanePtr MapInterface::GenerateRoadCorridorLane(const XodrLanePtr& xodr_lane) {
  LanePtr lane = std::make_shared<Lane>(xodr_lane);
  // center lines
  modules::geometry::Line center_line =
    roadgraph_->GetCenterLine(xodr_lane->get_id());
  lane->SetCenterLine(center_line);

  // polygons
  std::pair<PolygonPtr, bool> polygon_success =
    roadgraph_->ComputeXodrLanePolygon(xodr_lane->get_id());
  lane->SetPolygon(*polygon_success.first);
  return lane;
}

RoadPtr MapInterface::GenerateRoadCorridorRoad(const XodrRoadId& road_id) {
  XodrRoadPtr xodr_road = open_drive_map_->get_road(road_id);
  RoadPtr road = std::make_shared<Road>(xodr_road);
  Lanes lanes;
  for (auto& lane_section : xodr_road->get_lane_sections()) {
    for (auto& lane : lane_section->get_lanes()) {
      lanes[lane.first] = GenerateRoadCorridorLane(lane.second);
    }
  }
  road->SetLanes(lanes);
  return road;
}

void MapInterface::GenerateRoadCorridor(
  const XodrDrivingDirection& driving_direction,
  const std::vector<XodrRoadId>& road_ids) {
  std::size_t road_corridor_hash = RoadCorridor::GetHash(
    driving_direction, road_ids);
  
  // only compute if it has not been computed yet
  if (road_corridors_.count(road_corridor_hash) > 0)
    return;
  
  Roads roads;
  for (auto& road_id : road_ids) {
    roads[road_id] = GenerateRoadCorridorRoad(road_id);
  }
  // links can only be set once all roads have been calculated
  for (auto& road : roads) {
    // road successor
    XodrRoadId next_road_id = roadgraph_->GetNextRoad(road.first);
    road.second->SetNextRoad(roads[next_road_id]);
    for (auto& lane : road.second->GetLanes()) {
      // lane successor
      XodrLaneId next_lane_id = roadgraph_->GetNextLane(lane.first);
      lane.second->SetNextLane(roads[next_road_id]->GetLane(next_lane_id));

      // left and right lanes
      LanePtr left_lane = lane.second->GetLeftLane();
      LanePtr right_lane = lane.second->GetRightLane();
      lane.second->SetLeftLane(left_lane);
      lane.second->SetRightLane(right_lane);

      // set boundaries for lane
      XodrLaneId left_boundary_lane_id =
        roadgraph_->GetLeftBoundary(lane.first);
      XodrLaneId right_boundary_lane_id =
        roadgraph_->GetRightBoundary(lane.first);
      LanePtr left_lane_boundary = road.second->GetLane(
        left_boundary_lane_id);
      LanePtr right_lane_boundary = road.second->GetLane(
        right_boundary_lane_id);
      Boundary left_bound, right_bound;
      left_bound.SetLine(left_lane_boundary->get_line());
      left_bound.SetType(left_lane_boundary->get_road_mark());
      right_bound.SetLine(right_lane_boundary->get_line());
      right_bound.SetType(right_lane_boundary->get_road_mark());
      lane.second->SetLeftBoundary(left_bound);
      lane.second->SetRightBoundary(right_bound);
    }
  }

  RoadCorridorPtr road_corridor = std::make_shared<RoadCorridor>();
  road_corridor->SetRoads(roads);
  CalculateLaneCorridors(road_corridor);
  road_corridors_[road_corridor_hash] = road_corridor;
}

}  // namespace map
}  // namespace world
}  // namespace modules
