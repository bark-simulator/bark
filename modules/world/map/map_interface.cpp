// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include <math.h>
#include <random>
#include <memory>
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
  for (auto &road : open_drive_map_->GetRoads()) {
    for (auto &lane_section : road.second->GetLaneSections()) {
      for (auto &lane : lane_section->GetLanes()) {
        if (lane.second->GetLanePosition() == 0)
          continue;
        LineSegment lane_segment(*lane.second->GetLine().begin(),
                                 *(lane.second->GetLine().end() - 1));
        rtree_lane_.insert(
          std::make_pair(lane_segment, lane.second));
      }
    }
  }

  bounding_box_ = open_drive_map_->BoundingBox();
  return true;
}

bool MapInterface::FindNearestXodrLanes(const Point2d &point,
                                    const unsigned &num_lanes,
                                    std::vector<XodrLanePtr> &lanes,
                                    bool type_driving_only) const {
  std::vector<rtree_lane_value> results_n;
  if (type_driving_only) {
    rtree_lane_.query(
      boost::geometry::index::nearest(point, num_lanes) && boost::geometry::index::satisfies(IsLaneType),  // NOLINT
      std::back_inserter(results_n));
  } else {
    rtree_lane_.query(
      boost::geometry::index::nearest(point, num_lanes),
        std::back_inserter(results_n));
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

XodrLanePtr MapInterface::FindXodrLane(const Point2d& point) const {
  XodrLanePtr lane;
  std::vector<XodrLanePtr> nearest_lanes;
  // TODO(@esterle): parameter (20) auslagern
  if (!FindNearestXodrLanes(point, 20, nearest_lanes, false)) {
    return nullptr;
  }
  for (auto &close_lane : nearest_lanes) {
    if (IsInXodrLane(point, close_lane->GetId())) {
      lane = close_lane;
      return lane;
    }
  }
  return nullptr;
}

bool MapInterface::IsInXodrLane(const Point2d &point, XodrLaneId id) const {
  std::pair<vertex_t, bool> v = roadgraph_->GetVertexByLaneId(id);
  if (v.second) {
    auto polygon = roadgraph_->GetLaneGraph()[v.first].polygon;
    if (!polygon) {
      // found vertex has no polygon
      return false;
    } else {
      // found vertex has a polygon
      bool point_in_polygon = modules::geometry::Collide(*polygon, point);
      if (point_in_polygon) {
        return true;
      } else {
        return false;
      }
    }
  } else {
    // no vertex found
    return false;
  }
}


std::vector<PathBoundaries> MapInterface::ComputeAllPathBoundaries(
    const std::vector<XodrLaneId> &lane_ids) const
{
  std::vector<XodrLaneEdgeType> LANE_SUCCESSOR_EDGEs =
    {XodrLaneEdgeType::LANE_SUCCESSOR_EDGE};
  std::vector<std::vector<XodrLaneId>> all_paths =
    roadgraph_->FindAllPathsInSubgraph(LANE_SUCCESSOR_EDGEs, lane_ids);

  std::vector<PathBoundaries> all_path_boundaries;
  for (auto const &path : all_paths) {
    std::vector<std::pair<XodrLanePtr, XodrLanePtr>> path_boundaries;
    for (auto const &path_segment : path) {
      std::pair<XodrLanePtr, XodrLanePtr> lane_boundaries =
        roadgraph_->ComputeXodrLaneBoundaries(path_segment);
      path_boundaries.push_back(lane_boundaries);
    }
    all_path_boundaries.push_back(path_boundaries);
  }
  return all_path_boundaries;
}

std::pair<XodrLanePtr, bool> MapInterface::GetInnerNeighbor(
  const XodrLaneId lane_id) const {
  std::pair<XodrLaneId, bool> inner_neighbor =
  roadgraph_->GetInnerNeighbor(lane_id);
  if (inner_neighbor.second)
    return std::make_pair(roadgraph_->GetLanePtr(inner_neighbor.first), true);
  return std::make_pair(nullptr, false);
}

std::pair<XodrLanePtr, bool> MapInterface::GetOuterNeighbor(
  const XodrLaneId lane_id) const {
  std::pair<XodrLaneId, bool> outer_neighbor =
    roadgraph_->GetOuterNeighbor(lane_id);
  if (outer_neighbor.second)
    return std::make_pair(roadgraph_->GetLanePtr(outer_neighbor.first), true);
  return std::make_pair(nullptr, false);
}

std::vector<XodrLaneId> MapInterface::GetSuccessorLanes(
  const XodrLaneId lane_id) const {
  return roadgraph_->GetSuccessorLanes(lane_id);
}

void MapInterface::CalculateLaneCorridors(
  RoadCorridorPtr& road_corridor,
  const RoadPtr& road) {
  Lanes lanes = road->GetLanes();

  for (auto& lane : lanes) {
    // only add lane if it has not been added already
    if (road_corridor->GetLaneCorridor(lane.first) ||
        lane.second->GetLanePosition() == 0)
      continue;
    // only add if type is drivable
    if (lane.second->GetLaneType() != XodrLaneType::DRIVING)
      continue;

    LaneCorridorPtr lane_corridor = std::make_shared<LaneCorridor>();
    LanePtr current_lane = lane.second;
    float total_s = current_lane->GetCenterLine().Length();
    lane_corridor->SetCenterLine(current_lane->GetCenterLine());
    // lane_corridor->SetMergedPolygon(current_lane->GetPolygon());
    lane_corridor->SetLeftBoundary(
      current_lane->GetLeftBoundary().line_);
    lane_corridor->SetRightBoundary(
      current_lane->GetRightBoundary().line_);
    lane_corridor->SetLane(
      total_s,
      current_lane);

    // add initial lane
    road_corridor->SetLaneCorridor(current_lane->GetId(), lane_corridor);
    // std::cout << "Current Poly" << std::endl;
    // std::cout << current_lane->GetPolygon().ToArray() << std::endl;
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
      // std::cout << "New Poly" << std::endl;
      // std::cout << next_lane->GetPolygon().ToArray() << std::endl;
      // lane_corridor->GetMergedPolygon().ConcatenatePolygons(
      //   next_lane->GetPolygon());

      // TODO(@hart): use parameter
      geometry::Line simplified_line;
      boost::geometry::simplify(lane_corridor->GetCenterLine().obj_,
                                simplified_line.obj_, 0.1);
      simplified_line.RecomputeS();
      lane_corridor->SetCenterLine(simplified_line);
      // end of fix

      total_s = lane_corridor->GetCenterLine().Length();
      lane_corridor->SetLane(
        total_s,
        next_lane);
      // all following lanes should point to the same LaneCorridor
      road_corridor->SetLaneCorridor(next_lane->GetId(), lane_corridor);
    }

    // merged polygons
    PolygonPtr polygon = std::make_shared<modules::geometry::Polygon>();
    for (auto const &p : lane_corridor->GetLeftBoundary()) {
      polygon->AddPoint(p);
    }
    auto reversed_outer = lane_corridor->GetRightBoundary();
    reversed_outer.Reverse();
    for (auto const &p : reversed_outer) {
      polygon->AddPoint(p);
    }
    // Polygons need to be closed!
    polygon->AddPoint(*(lane_corridor->GetLeftBoundary().begin()));
    lane_corridor->SetMergedPolygon(*polygon);
  }
}

void MapInterface::CalculateLaneCorridors(
  RoadCorridorPtr& road_corridor,
  const XodrRoadId& road_id) {
  RoadPtr first_road = road_corridor->GetRoads()[road_id];
  CalculateLaneCorridors(road_corridor, first_road);
  for (auto& road : road_corridor->GetRoads()) {
    CalculateLaneCorridors(road_corridor, road.second);
  }
}

LanePtr MapInterface::GenerateRoadCorridorLane(const XodrLanePtr& xodr_lane) {
  LanePtr lane = std::make_shared<Lane>(xodr_lane);
  // polygons
  if (xodr_lane->GetLanePosition() != 0) {
    std::pair<PolygonPtr, bool> polygon_success =
      roadgraph_->ComputeXodrLanePolygon(xodr_lane->GetId());
    // Cannot compute polygon for planview!!
    if (polygon_success.second)
      lane->SetPolygon(*polygon_success.first);
  }
  return lane;
}

RoadPtr MapInterface::GenerateRoadCorridorRoad(const XodrRoadId& road_id) {
  XodrRoadPtr xodr_road = open_drive_map_->GetRoad(road_id);
  RoadPtr road = std::make_shared<Road>(xodr_road);
  Lanes lanes;
  for (auto& lane_section : xodr_road->GetLaneSections()) {
    for (auto& lane : lane_section->GetLanes()) {
      // TODO(@hart): only add driving lanes
      // if (lane.second->GetLaneType() == XodrLaneType::DRIVING)
      lanes[lane.first] = GenerateRoadCorridorLane(lane.second);
    }
  }
  road->SetLanes(lanes);
  return road;
}

void MapInterface::GenerateRoadCorridor(
  const std::vector<XodrRoadId>& road_ids,
  const XodrDrivingDirection& driving_direction) {
  std::size_t road_corridor_hash = RoadCorridor::GetHash(
    driving_direction, road_ids);

  // only compute if it has not been computed yet
  if (road_corridors_.count(road_corridor_hash) > 0)
    return;

  Roads roads;
  for (auto& road_id : road_ids)
    roads[road_id] = GenerateRoadCorridorRoad(road_id);

  // links can only be set once all roads have been calculated
  int count = 0;
  for (auto& road_id : road_ids) {
    // road successor
    RoadPtr next_road;
    if (count < road_ids.size() - 1)
      next_road = roads[road_ids[++count]];
    roads[road_id]->SetNextRoad(next_road);
    for (auto& lane : roads[road_id]->GetLanes()) {
      // lane successor
      // auto local_driving_direction = driving_direction;
      // if (lane.second->GetDrivingDirection() != driving_direction)
      //   local_driving_direction = lane.second->GetDrivingDirection();
        
      if (count + 1 <= road_ids.size()) {
        std::vector<XodrRoadId> vec;
        std::copy(
          road_ids.begin() + count,
          road_ids.begin() + count + 1,
          std::back_inserter(vec));

        // for (auto v: vec)
        //   std::cout << v << std::endl;
        
        std::pair<XodrLaneId, bool> next_lane =
          roadgraph_->GetNextLane(vec, lane.first);
        if (next_lane.second && next_road)
          lane.second->SetNextLane(next_road->GetLane(next_lane.first));
      }

      // left and right lanes
      LanePtr left_lane, right_lane;
      std::pair<XodrLaneId, bool> left_lane_id =
        roadgraph_->GetLeftLane(lane.first, lane.second->GetDrivingDirection());
      if (left_lane_id.second) {
        left_lane = roads[road_id]->GetLane(left_lane_id.first);
        lane.second->SetLeftLane(left_lane);
      }

      std::pair<XodrLaneId, bool> right_lane_id =
        roadgraph_->GetRightLane(lane.first, lane.second->GetDrivingDirection());
      if (right_lane_id.second) {
        right_lane = roads[road_id]->GetLane(right_lane_id.first);
        lane.second->SetRightLane(right_lane);
      }

      // set boundaries for lane
      std::pair<XodrLaneId, bool> left_boundary_lane_id =
        roadgraph_->GetLeftBoundary(lane.first, lane.second->GetDrivingDirection());
      if (left_boundary_lane_id.second) {
        LanePtr left_lane_boundary = roads[road_id]->GetLane(
          left_boundary_lane_id.first);
        Boundary left_bound;
        left_bound.SetLine(left_lane_boundary->GetLine());
        left_bound.SetType(left_lane_boundary->GetRoad_mark());
        lane.second->SetLeftBoundary(left_bound);
      }
      std::pair<XodrLaneId, bool> right_boundary_lane_id =
        roadgraph_->GetRightBoundary(lane.first, lane.second->GetDrivingDirection());
      if (right_boundary_lane_id.second) {
        LanePtr right_lane_boundary = roads[road_id]->GetLane(
          right_boundary_lane_id.first);
        Boundary right_bound;
        right_bound.SetLine(right_lane_boundary->GetLine());
        right_bound.SetType(right_lane_boundary->GetRoad_mark());
        lane.second->SetRightBoundary(right_bound);
      }

      // compute center line
      if (left_boundary_lane_id.second && right_boundary_lane_id.second)
        lane.second->SetCenterLine(
          ComputeCenterLine(lane.second->GetLeftBoundary().line_,
          lane.second->GetRightBoundary().line_));
    }
  }

  if (roads.size() == 0)
    return;
  RoadCorridorPtr road_corridor = std::make_shared<RoadCorridor>();
  road_corridor->SetRoads(roads);
  CalculateLaneCorridors(road_corridor, road_ids[0]);
  road_corridor->ComputeRoadPolygon();
  road_corridor->SetRoadIds(road_ids);
  road_corridors_[road_corridor_hash] = road_corridor;
}

RoadCorridorPtr MapInterface::GenerateRoadCorridor(
  const modules::geometry::Point2d& start_point,
  const modules::geometry::Polygon& goal_region) {
  std::vector<XodrLanePtr> lanes;
  XodrLaneId goal_lane_id;
  bool nearest_start_lane_found = FindNearestXodrLanes(start_point, 1, lanes);
  bool nearest_goal_lane_found = XodrLaneIdAtPolygon(goal_region,
                                                     goal_lane_id);
  if (!nearest_start_lane_found || !nearest_goal_lane_found) {
    LOG(INFO) << "Could not generate road corridor based on geometric start and goal definitions.";  // NOLINT
    return nullptr;
  }

  const auto start_lane_id = lanes.at(0)->GetId();
  const XodrDrivingDirection driving_direction =
    lanes.at(0)->GetDrivingDirection();

  std::vector<XodrRoadId> road_ids;
  std::vector<XodrLaneId> lane_ids = roadgraph_->FindDrivableLanePath(
    start_lane_id, goal_lane_id);
  for (auto lid : lane_ids) {
    std::pair<vertex_t, bool> v_des = roadgraph_->GetVertexByLaneId(lid);
    XodrLaneVertex lv = roadgraph_->GetVertex(v_des.first);
    road_ids.push_back(lv.road_id);
    // std::cout << "lane_id: " << lid << ", road_id: ";
    // std::cout << lv.road_id << std::endl;
  }
  GenerateRoadCorridor(road_ids, driving_direction);
  return GetRoadCorridor(road_ids, driving_direction);
}

RoadCorridorPtr MapInterface::GenerateRoadCorridor(const XodrRoadId& start_road_id,
    const XodrRoadId& end_road_id) {
    std::vector<XodrRoadId> road_ids = roadgraph_->FindRoadPath(start_road_id,
                                                end_road_id);

    std::pair<std::vector<XodrDrivingDirection>, bool> directions =
               roadgraph_->GetDrivingDirectionsForRoadId(start_road_id);
    if (!directions.second) {
      LOG(ERROR) << "No lanes for start road id " << start_road_id << " found.";
      return nullptr;
    }
    XodrDrivingDirection driving_direction = directions.first.at(0);
    GenerateRoadCorridor(road_ids, driving_direction);
    return GetRoadCorridor(road_ids, driving_direction);
}

bool MapInterface::XodrLaneIdAtPolygon(
  const modules::geometry::Polygon&  polygon,
  XodrLaneId& found_lane_id) const {
  modules::geometry::Point2d goal_center(polygon.center_(0),
                                         polygon.center_(1));
  std::vector<opendrive::XodrLanePtr> nearest_lanes;
  if (FindNearestXodrLanes(goal_center, 1, nearest_lanes)) {
      found_lane_id = nearest_lanes[0]->GetId();
      return true;
  }
  LOG(INFO) << "No matching lane for goal definition found";
  return false;
}

RoadPtr MapInterface::GetNextRoad(
  const XodrRoadId& current_road_id,
  const Roads& roads,
  const std::vector<XodrRoadId>& road_ids) const {
  auto it = std::find(
    road_ids.begin(),
    road_ids.end(),
    current_road_id);
  if (road_ids.back() == current_road_id)
    return nullptr;
  return roads.at(*std::next(it, 1));
}

}  // namespace map
}  // namespace world
}  // namespace modules
