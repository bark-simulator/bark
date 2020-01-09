// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <utility>
#include "modules/world/map/local_map.hpp"
#include "modules/world/map/map_interface.hpp"
#include "modules/geometry/polygon.hpp"

namespace modules {
namespace world {
namespace map {

using modules::world::opendrive::LanePtr;
using geometry::FindNearestIdx;
using geometry::distance;
using models::dynamic::StateDefinition;

LaneId LocalMap::GoalLaneIdFromPolygon(
  const modules::geometry::Polygon& goal_polygon) {
  modules::geometry::Point2d goal_center(goal_polygon.center_(0),
                                         goal_polygon.center_(1));
  std::vector<opendrive::LanePtr> nearest_lanes;

  if (map_interface_->FindNearestLanes(goal_center, 1, nearest_lanes)) {
      return nearest_lanes[0]->get_id();
  }
  printf("No matching lane for goal definition found. Defaulting to LaneId 0.");
  return LaneId(0);
}

LanePtr LocalMap::FindLane(const Point2d& point) const {
  LanePtr lane = map_interface_->FindLane(point);
  return lane;
}

bool LocalMap::HasCorrectDrivingDirection(const State& state) const {
  geometry::Point2d position(
    state(StateDefinition::X_POSITION),
    state(StateDefinition::Y_POSITION));
  float orientation = state(StateDefinition::THETA_POSITION);
  bool correct_direction = map_interface_->HasCorrectDrivingDirection(
    position,
    orientation);
  return correct_direction;
}

bool LocalMap::Generate(Point2d point) {
  if (map_interface_ == nullptr) {
    return false;
  }
  driving_corridor_ = DrivingCorridor();

  goal_lane_id_ = GoalLaneIdFromPolygon(
    goal_definition_->get_shape());
  std::vector<LanePtr> lanes;
  map_interface_->FindNearestLanes(point, 1, lanes);
  LanePtr current_lane = lanes.at(0);

  driving_corridor_ = map_interface_->ComputeDrivingCorridorFromStartToGoal(
    current_lane->get_id(),
    goal_lane_id_);

  if (!driving_corridor_.computed) {
    return false;
  }
  return true;
}

bool LocalMap::RecalculateDrivingCorridor(const Point2d &point) {
  if (map_interface_ == nullptr) {
    return false;
  }

  std::vector<LanePtr> lanes;
  map_interface_->FindNearestLanes(point, 1, lanes);
  LanePtr current_lane = lanes.at(0);

  driving_corridor_ = map_interface_->ComputeDrivingCorridorFromStartToGoal(
    current_lane->get_id(), goal_lane_id_);
  if (!driving_corridor_.computed) {
    // No direct corridor to the goal lane exists
    // try finding a corridor parallel to the goal lane
    driving_corridor_ = map_interface_->ComputeDrivingCorridorParallelToGoal(
      current_lane->get_id(), goal_lane_id_);
  }
  if (!driving_corridor_.computed) {
    // TODO(@AKreutz): Maybe it is still possible to generate
    // a driving corridor in this case?
    LOG(ERROR) << "Driving corridor generation failed, goal lane " << goal_lane_id_  // NOLINT
               << " is not reachable from lane " << current_lane->get_id()
               << ", neither are any of its neighboring lanes" << std::endl;
    return false;
  } else {
    return true;
  }
}


Line LocalMap::CalculateLineHorizon(const Line& line,
                            const Point2d& p,
                            double horizon) {
  // TODO(@hart): do not access via member obj_
  Line new_line;
  int num_points = line.obj_.size();
  if (num_points == 0) {
    return new_line;
  }
  int nearest_idx = FindNearestIdx(line, p);
  new_line.add_point(line.obj_.at(nearest_idx));
  if (nearest_idx == num_points-1) {
    return new_line;
  }

  double s = 0.0;
  for (int idx = nearest_idx; idx < num_points - 1; idx++) {
    // TODO(@hart): in case of non circular routes.. criteria to abort?
    int idx_ = idx % num_points;  // in case of circle
    int idx_next = (idx+1) % num_points;  // in case of circle
    double d = distance(line.obj_.at(idx_), line.obj_.at(idx_next));
    new_line.add_point(line.obj_.at(idx_next));
    s += d;
    if ( s >= horizon )
      return new_line;
  }
  return new_line;
}

bool LocalMap::ComputeHorizonCorridor(const Point2d& p, double horizon) {
  horizon_driving_corridor_.inner =
    CalculateLineHorizon(driving_corridor_.inner, p, horizon);
  horizon_driving_corridor_.outer =
    CalculateLineHorizon(driving_corridor_.outer, p, horizon);
  if (horizon_driving_corridor_.inner.obj_.size() > 0 &&
      horizon_driving_corridor_.outer.obj_.size() > 0) {
    horizon_driving_corridor_.center =
      ComputeCenterLine(horizon_driving_corridor_.inner,
                        horizon_driving_corridor_.outer);
    return true;
  }
  return false;
}

LocalMap *LocalMap::Clone() const {
  LocalMap *new_local_map = new LocalMap(*this);
  return new_local_map;
}

}  // namespace map
}  // namespace world
}  // namespace modules