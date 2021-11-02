// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/world/map/map_interface.hpp"
#include <math.h>
#include <boost/tokenizer.hpp>
#include <memory>
#include <random>

namespace bark {
namespace world {
namespace map {

using LineSegment = boost::geometry::model::segment<Point2d>;
bool MapInterface::interface_from_opendrive(
    const OpenDriveMapPtr& open_drive_map) {
  open_drive_map_ = open_drive_map;

  RoadgraphPtr roadgraph(new Roadgraph());
  roadgraph->Generate(open_drive_map);
  roadgraph_ = roadgraph;

  rtree_lane_.clear();
  for (auto& road : open_drive_map_->GetRoads()) {
    for (auto& lane_section : road.second->GetLaneSections()) {
      for (auto& lane : lane_section->GetLanes()) {
        if (lane.second->GetLanePosition() == 0) continue;
        LineSegment lane_segment(*lane.second->GetLine().begin(),
                                 *(lane.second->GetLine().end() - 1));
        rtree_lane_.insert(std::make_pair(lane_segment, lane.second));
      }
    }
  }

  bounding_box_ = open_drive_map_->BoundingBox();
  return true;
}

bool MapInterface::interface_from_csvtable(const std::string csvfile) {
  // Read map data
  std::ifstream file(csvfile);
  if (!file.is_open()) {
    LOG(ERROR) << "Error reading mapfile " << csvfile;
    return false;
  }
  std::vector<double> cx, cy, lx, ly, rx, ry;
  typedef boost::tokenizer<boost::escaped_list_separator<char>> Tokenizer;
  std::string line;
  std::vector<std::string> row;
  bool toprow = true;
  while (getline(file, line)) {
    if (!toprow) {
      Tokenizer tok(line);
      row.assign(tok.begin(), tok.end());
      cx.push_back(stod(row[1]));
      cy.push_back(stod(row[2]));
      rx.push_back(stod(row[3]));
      ry.push_back(stod(row[4]));
      lx.push_back(stod(row[5]));
      ly.push_back(stod(row[6]));
    } else {
      toprow = false;
    }
  }
  const int nr_points = cx.size();

  // Generate centerline
  using bark::geometry::Line;
  using bark::geometry::Point2d;
  Line centerline;
  for (int idx = 0; idx < nr_points; ++idx) {
    centerline.AddPoint(Point2d(cx[idx], cy[idx]));
  }

  // Generate road polygon
  using bark::geometry::Polygon;
  Polygon lanepoly;
  for (int idx = 0; idx < nr_points; ++idx) {
    lanepoly.AddPoint(Point2d(lx[idx], ly[idx]));
  }
  for (int idx = nr_points-1; idx >= 0; --idx) {  // reverse
    lanepoly.AddPoint(Point2d(rx[idx], ry[idx]));
  }
  // lanepoly.correct(); //@todo do we need this?

  // Generate Lane
  double speed = 30 / 3.6;
  int laneid = 0;
  XodrLanePtr xodrlane = std::make_shared<XodrLane>();
  xodrlane->SetId(laneid);
  xodrlane->SetLanePosition(-1);  //@todo how to assign? Type: XodrLanePosition
  // xodrlane->link_; //not needed
  xodrlane->SetLine(centerline);
  // xodrlane->junction_id_ //not needed
  xodrlane->SetIsInJunction(false);
  xodrlane->SetLaneType(XodrLaneType::DRIVING);
  xodrlane->SetDrivingDirection(XodrDrivingDirection::FORWARD);
  // xodrlane->road_mark_ // @todo do we need this to be assigned= Type:
  // XodrRoadMark
  xodrlane->SetSpeed(speed);
  // xodrlane->lane_count//@todo how to assign?

  LanePtr lane = std::make_shared<Lane>(xodrlane);
  // lane->left_lane_ = nullptr; //@todo do we have to assign?
  // lane->right_lane_ = nullptr; //@todo do we have to assign?
  // lane->next_lane_ = nullptr; //@todo do we have to assign?
  lane->center_line_ = centerline;
  lane->polygon_ = lanepoly;
  // Boundary left_boundary_; //@todo do we need this?
  // Boundary right_boundary_; //@todo do we need this?

  // Generate LaneCorridorPtr!
  LaneCorridorPtr lanecorridor = std::make_shared<LaneCorridor>();
  lanecorridor->lanes_[0] = lane;  // s_end: @todo what is the key? is zero ok??
  lanecorridor->center_line_ = centerline;
  lanecorridor->fine_center_line_ = centerline;
  lanecorridor->merged_polygon_ = lanepoly;
  // Line left_boundary_; //@todo do we need this?
  // Line right_boundary_; //@todo do we need this?

  // Generate PlanView
  // @todo if we need the planview make a constructor from line!
  PlanViewPtr xodrplanview = std::make_shared<PlanView>();

  // Generate XodrLaneSection
  double s = 0;
  XodrLaneSectionPtr xodrlanesection = std::make_shared<XodrLaneSection>(s);
  xodrlanesection->AddLane(lane);

  // Generate XodrRoad
  int roadid = 0;
  XodrRoadPtr xodrroad = std::make_shared<XodrRoad>();
  xodrroad->SetId(roadid);
  xodrroad->SetName("dummy_name");
  // xodrroad->SetLink(); //not needed
  xodrroad->SetPlanView(xodrplanview);
  xodrroad->AddLaneSection({xodrlanesection});

  // Generate Road
  RoadPtr road = std::make_shared<Road>(xodrroad);
  road->next_road_ = nullptr;
  road->lanes_[laneid] = lane;

  // Generate road corridor
  RoadCorridorPtr rc = std::make_shared<RoadCorridor>();
  rc->roads_[roadid] = road;
  rc->road_polygon_ = lanepoly;
  rc->unique_lane_corridors_ = {lanecorridor};
  rc->road_ids_ = {roadid};
  rc->driving_direction_ = XodrDrivingDirection::FORWARD;
  rc->lane_corridors_[laneid] = lanecorridor;

  // Generate roadgraph
  RoadgraphPtr roadgraph(new Roadgraph());
  roadgraph->AddLane(roadid, xodrlane);
  bark::world::map::PolygonPtr lanepolyptr =
      std::make_shared<Polygon>(lanepoly);
  bool lanepoly_set = roadgraph->SetPolygonForVertexFromId(laneid, lanepolyptr);
  if (!lanepoly_set) {
    return false;
  }

  // Generate lane rtree
  rtree_lane_.clear();
  using LineSegment = boost::geometry::model::segment<Point2d>;
  LineSegment lane_segment(
      *centerline.begin(),
      *(centerline.end() - 1));  //@todo ERROR! should be the boundary here, but
                                 // which? I did not model the planview
  rtree_lane_.insert(std::make_pair(lane_segment, lane));

  // Generate bounding box
  boost::geometry::model::box<Point2d> box;
  boost::geometry::envelope(lanepoly.obj_, box);
  boost::geometry::correct(box);
  bounding_box_ = std::make_pair(
      Point2d(boost::geometry::get<boost::geometry::min_corner, 0>(box),
              bg::get<bg::min_corner, 1>(box)),
      Point2d(boost::geometry::get<boost::geometry::max_corner, 0>(box),
              bg::get<bg::max_corner, 1>(box)));

  // Generate (dummy) Open Drive Map
  OpenDriveMapPtr open_drive_map = std::make_shared<OpenDriveMap>();
  open_drive_map->AddRoad(xodrroad);

  // Generate map interface
  open_drive_map_ = open_drive_map;
  road_from_csvtable_ = true;
  roadgraph_ = roadgraph;
  // rtree_lane_ assigned above

  // TODO THIS IS A HACK!!!
  std::vector<XodrRoadId> road_ids = {static_cast<XodrRoadId>(roadid)};
  XodrDrivingDirection driving_direction = XodrDrivingDirection::FORWARD;
  std::size_t road_corridor_hash = RoadCorridor::GetHash(driving_direction, road_ids);

  road_corridors_[road_corridor_hash] = rc;
  // bounding_box_ assigned above
  return true;
}

bool MapInterface::FindNearestXodrLanes(const Point2d& point,
                                        const unsigned& num_lanes,
                                        std::vector<XodrLanePtr>& lanes,
                                        bool type_driving_only) const {
  std::vector<rtree_lane_value> results_n;
  if (type_driving_only) {
    rtree_lane_.query(
        boost::geometry::index::nearest(point, num_lanes) &&
            boost::geometry::index::satisfies(IsLaneType),  // NOLINT
        std::back_inserter(results_n));
  } else {
    rtree_lane_.query(boost::geometry::index::nearest(point, num_lanes),
                      std::back_inserter(results_n));
  }
  if (results_n.empty()) {
    return false;
  }
  lanes.clear();
  for (auto& result : results_n) {
    lanes.push_back(result.second);
  }
  return true;
}

XodrLanePtr MapInterface::FindXodrLane(const Point2d& point) const {
  XodrLanePtr lane;
  std::vector<XodrLanePtr> nearest_lanes;
  if (!FindNearestXodrLanes(point, num_points_nearest_lane_, nearest_lanes,
                            false)) {
    return nullptr;
  }
  for (auto& close_lane : nearest_lanes) {
    if (IsInXodrLane(point, close_lane->GetId())) {
      lane = close_lane;
      return lane;
    }
  }
  return nullptr;
}

bool MapInterface::IsInXodrLane(const Point2d& point, XodrLaneId id) const {
  std::pair<vertex_t, bool> v = roadgraph_->GetVertexByLaneId(id);
  if (v.second) {
    auto polygon = roadgraph_->GetLaneGraph()[v.first].polygon;
    if (!polygon) {
      // found vertex has no polygon
      return false;
    } else {
      // found vertex has a polygon
      bool point_in_polygon = bark::geometry::Collide(*polygon, point);
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
    const std::vector<XodrLaneId>& lane_ids) const {
  std::vector<XodrLaneEdgeType> LANE_SUCCESSOR_EDGEs = {
      XodrLaneEdgeType::LANE_SUCCESSOR_EDGE};
  std::vector<std::vector<XodrLaneId>> all_paths =
      roadgraph_->FindAllPathsInSubgraph(LANE_SUCCESSOR_EDGEs, lane_ids);

  std::vector<PathBoundaries> all_path_boundaries;
  for (auto const& path : all_paths) {
    std::vector<std::pair<XodrLanePtr, XodrLanePtr>> path_boundaries;
    for (auto const& path_segment : path) {
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

void MapInterface::CalculateLaneCorridors(RoadCorridorPtr& road_corridor,
                                          const RoadPtr& road) {
  Lanes lanes = road->GetLanes();

  for (auto& lane : lanes) {
    // only add lane if it has not been added already
    if (road_corridor->GetLaneCorridor(lane.first) ||
        lane.second->GetLanePosition() == 0)
      continue;
    // only add if type is drivable
    if (lane.second->GetLaneType() != XodrLaneType::DRIVING) continue;

    LaneCorridorPtr lane_corridor = std::make_shared<LaneCorridor>();
    LanePtr current_lane = lane.second;
    double total_s = current_lane->GetCenterLine().Length();
    lane_corridor->SetCenterLine(current_lane->GetCenterLine());
    lane_corridor->SetFineCenterLine(current_lane->GetCenterLine());
    // lane_corridor->SetMergedPolygon(current_lane->GetPolygon());
    lane_corridor->SetLeftBoundary(current_lane->GetLeftBoundary().line_);
    lane_corridor->SetRightBoundary(current_lane->GetRightBoundary().line_);
    lane_corridor->SetLane(total_s, current_lane);

    // add initial lane
    road_corridor->SetLaneCorridor(current_lane->GetId(), lane_corridor);
    // std::cout << "Current Poly" << std::endl;
    // std::cout << current_lane->GetPolygon().ToArray() << std::endl;
    LanePtr next_lane = current_lane;
    for (;;) {
      next_lane = next_lane->GetNextLane();
      if (next_lane == nullptr) break;
      Line new_center = bark::geometry::ConcatenateLinestring(
          lane_corridor->GetCenterLine(), next_lane->GetCenterLine());
      lane_corridor->SetCenterLine(new_center);
      lane_corridor->SetFineCenterLine(new_center);

      Line new_left = bark::geometry::ConcatenateLinestring(
          lane_corridor->GetLeftBoundary(), next_lane->GetLeftBoundary().line_);
      lane_corridor->SetLeftBoundary(new_left);

      Line new_right = bark::geometry::ConcatenateLinestring(
          lane_corridor->GetRightBoundary(),
          next_lane->GetRightBoundary().line_);
      lane_corridor->SetRightBoundary(new_right);
      // std::cout << "New Poly" << std::endl;
      // std::cout << next_lane->GetPolygon().ToArray() << std::endl;
      // lane_corridor->GetMergedPolygon().ConcatenatePolygons(
      //   next_lane->GetPolygon());

      total_s = lane_corridor->GetCenterLine().Length();
      lane_corridor->SetLane(total_s, next_lane);
      // all following lanes should point to the same LaneCorridor
      road_corridor->SetLaneCorridor(next_lane->GetId(), lane_corridor);
    }

    // \note \kessler: setting max_simplification_dist_ too small can yield
    // self-intersecting road polygons. why? in curves on the inner radius due
    // to sampling inaccuracy the boundaries of two segments can overlap. The
    // code below just copies each point to the lane polygon without checking
    // this.

    Line simplf_center =
        Simplify(lane_corridor->GetCenterLine(), max_simplification_dist_);
    lane_corridor->SetCenterLine(simplf_center);

    Line simplf_fine_center =
        Simplify(lane_corridor->GetFineCenterLine(), 0.001);
    lane_corridor->SetFineCenterLine(simplf_fine_center);

    Line simplf_right =
        Simplify(lane_corridor->GetRightBoundary(), max_simplification_dist_);
    lane_corridor->SetRightBoundary(simplf_right);

    Line simplf_left =
        Simplify(lane_corridor->GetLeftBoundary(), max_simplification_dist_);
    lane_corridor->SetLeftBoundary(simplf_left);

    // TODO hier ist der self intersection bug: magic number 0.5 durch sampling
    // distance ersetzen!!!

    // merged polygons
    PolygonPtr polygon = std::make_shared<bark::geometry::Polygon>();

    // \todo (\kessler): before adding a point, check if this produces a
    // self-intersection.
    for (auto const& p : lane_corridor->GetLeftBoundary()) {
      polygon->AddPoint(p);
    }
    auto reversed_outer = lane_corridor->GetRightBoundary();
    reversed_outer.Reverse();
    for (auto const& p : reversed_outer) {
      polygon->AddPoint(p);
    }
    // Polygons need to be closed!
    polygon->AddPoint(*(lane_corridor->GetLeftBoundary().begin()));
    boost::geometry::correct(polygon->obj_);

    if (!polygon->Valid()) {
      LOG(ERROR) << "Producing a non-valid lane corridor for lane id = "
                 << lane.first;
    }

    lane_corridor->SetMergedPolygon(*polygon);
  }
}

void MapInterface::CalculateLaneCorridors(RoadCorridorPtr& road_corridor,
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
    if (polygon_success.second) lane->SetPolygon(*polygon_success.first);
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
  std::size_t road_corridor_hash =
      RoadCorridor::GetHash(driving_direction, road_ids);

  // only compute if it has not been computed yet
  if (road_corridors_.count(road_corridor_hash) > 0) return;

  Roads roads;
  for (auto& road_id : road_ids)
    roads[road_id] = GenerateRoadCorridorRoad(road_id);

  // links can only be set once all roads have been calculated
  int count = 0;
  for (auto& road_id : road_ids) {
    // road successor
    RoadPtr next_road;
    if (count < road_ids.size() - 1) next_road = roads[road_ids[++count]];
    roads[road_id]->SetNextRoad(next_road);
    for (auto& lane : roads[road_id]->GetLanes()) {
      // lane successor
      // auto local_driving_direction = driving_direction;
      // if (lane.second->GetDrivingDirection() != driving_direction)
      //   local_driving_direction = lane.second->GetDrivingDirection();

      if (count + 1 <= road_ids.size()) {
        std::vector<XodrRoadId> vec;
        std::copy(road_ids.begin() + count, road_ids.begin() + count + 1,
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
      std::pair<XodrLaneId, bool> left_lane_id = roadgraph_->GetLeftLane(
          lane.first, lane.second->GetDrivingDirection());
      if (left_lane_id.second) {
        left_lane = roads[road_id]->GetLane(left_lane_id.first);
        lane.second->SetLeftLane(left_lane);
      }

      std::pair<XodrLaneId, bool> right_lane_id = roadgraph_->GetRightLane(
          lane.first, lane.second->GetDrivingDirection());
      if (right_lane_id.second) {
        right_lane = roads[road_id]->GetLane(right_lane_id.first);
        lane.second->SetRightLane(right_lane);
      }

      // set boundaries for lane
      std::pair<XodrLaneId, bool> left_boundary_lane_id =
          roadgraph_->GetLeftBoundary(lane.first,
                                      lane.second->GetDrivingDirection());
      if (left_boundary_lane_id.second) {
        LanePtr left_lane_boundary =
            roads[road_id]->GetLane(left_boundary_lane_id.first);
        Boundary left_bound;
        left_bound.SetLine(left_lane_boundary->GetLine());
        left_bound.SetType(left_lane_boundary->GetRoad_mark());
        lane.second->SetLeftBoundary(left_bound);
      }
      std::pair<XodrLaneId, bool> right_boundary_lane_id =
          roadgraph_->GetRightBoundary(lane.first,
                                       lane.second->GetDrivingDirection());
      if (right_boundary_lane_id.second) {
        LanePtr right_lane_boundary =
            roads[road_id]->GetLane(right_boundary_lane_id.first);
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

  if (roads.size() == 0) return;
  RoadCorridorPtr road_corridor = std::make_shared<RoadCorridor>();
  road_corridor->SetRoads(roads);
  CalculateLaneCorridors(road_corridor, road_ids[0]);
  road_corridor->ComputeRoadPolygon();
  if (full_junction_area_) {
    for (auto& junction : road_corridor->GetJunctionIds()) {
      const Polygon poly = ComputeJunctionArea(junction);
      road_corridor->AddPolygonToRoadCorridor(std::move(poly));
    }
  }
  road_corridor->SetRoadIds(road_ids);
  road_corridor->SetDrivingDirection(driving_direction);
  road_corridors_[road_corridor_hash] = road_corridor;
}

RoadCorridorPtr MapInterface::GenerateRoadCorridor(
    const bark::geometry::Point2d& start_point,
    const bark::geometry::Polygon& goal_region) {
  std::vector<XodrLanePtr> lanes;
  XodrLaneId goal_lane_id;
  bool nearest_start_lane_found = FindNearestXodrLanes(start_point, 1, lanes);
  bool nearest_goal_lane_found = XodrLaneIdAtPolygon(goal_region, goal_lane_id);
  if (!nearest_start_lane_found || !nearest_goal_lane_found) {
    LOG(INFO) << "Could not generate road corridor based on geometric start "
                 "and goal definitions.";  // NOLINT
    return nullptr;
  }

  const auto start_lane_id = lanes.at(0)->GetId();
  const XodrDrivingDirection driving_direction =
      lanes.at(0)->GetDrivingDirection();

  std::vector<XodrRoadId> road_ids;
  std::vector<XodrLaneId> lane_ids =
      roadgraph_->FindDrivableLanePath(start_lane_id, goal_lane_id);
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

RoadCorridorPtr MapInterface::GenerateRoadCorridor(
    const XodrRoadId& start_road_id, const XodrRoadId& end_road_id) {
  std::vector<XodrRoadId> road_ids =
      roadgraph_->FindRoadPath(start_road_id, end_road_id);

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

bool MapInterface::XodrLaneIdAtPolygon(const bark::geometry::Polygon& polygon,
                                       XodrLaneId& found_lane_id) const {
  bark::geometry::Point2d goal_center(polygon.center_(0), polygon.center_(1));
  std::vector<opendrive::XodrLanePtr> nearest_lanes;
  if (FindNearestXodrLanes(goal_center, 1, nearest_lanes)) {
    found_lane_id = nearest_lanes[0]->GetId();
    return true;
  }
  LOG(INFO) << "No matching lane for goal definition found";
  return false;
}

RoadPtr MapInterface::GetNextRoad(
    const XodrRoadId& current_road_id, const Roads& roads,
    const std::vector<XodrRoadId>& road_ids) const {
  auto it = std::find(road_ids.begin(), road_ids.end(), current_road_id);
  if (road_ids.back() == current_road_id) return nullptr;
  return roads.at(*std::next(it, 1));
}

bark::geometry::Polygon MapInterface::ComputeJunctionArea(
    uint32_t junction_id) {
  PolygonPtr poly = roadgraph_->ComputeJunctionArea(junction_id);
  return *poly.get();
}

}  // namespace map
}  // namespace world
}  // namespace bark
