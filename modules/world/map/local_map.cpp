// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <utility>
#include "modules/world/map/local_map.hpp"
#include "modules/world/map/map_interface.hpp"

namespace modules {
namespace world {
namespace map {

using modules::world::opendrive::LanePtr;
using geometry::FindNearestIdx;
using geometry::distance;
using models::dynamic::StateDefinition;

LaneId LocalMap::GoalLaneIdFromGoalDefinitionPolygon(const GoalDefinitionPolygon& goal_definition) {
  modules::geometry::Point2d goal_center(goal_definition.get_shape().center_(0),
                                         goal_definition.get_shape().center_(1));
  std::vector<opendrive::LanePtr> nearest_lanes;

  if(map_interface_->FindNearestLanes(goal_center, 1, nearest_lanes)) {
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
  geometry::Point2d position(state(StateDefinition::X_POSITION), state(StateDefinition::Y_POSITION));
  float orientation = state(StateDefinition::THETA_POSITION);

  bool correct_direction = map_interface_->HasCorrectDrivingDirection(position, orientation);
  return correct_direction;
}

bool LocalMap::Generate(Point2d point) {
  if(map_interface_ == nullptr) {
    return false;
  }
  driving_corridor_ = DrivingCorridor();

  auto goal_definition_polygon = std::dynamic_pointer_cast<GoalDefinitionPolygon>(goal_definition_);
  if (!goal_definition_polygon) {
    return false; //< todo: handle this better
  }

  goal_lane_id_ = GoalLaneIdFromGoalDefinitionPolygon(*goal_definition_polygon);

  std::vector<LanePtr> lanes;
  map_interface_->FindNearestLanes(point, 1, lanes);
  LanePtr current_lane = lanes.at(0);

  driving_corridor_ = map_interface_->ComputeDrivingCorridorFromStartToGoal(current_lane->get_id(), goal_lane_id_);
  if (!driving_corridor_.computed) {
    return false;
  }

  return true;
}



Line LocalMap::CalculateLineHorizon(const Line& line,
                            const Point2d& p,
                            double horizon) {
  // TODO(@hart): do not access via member obj_
  
  Line new_line;
  int num_points = line.obj_.size();
  if(num_points == 0) {
    return new_line;
  }
  int nearest_idx = FindNearestIdx(line, p);
  new_line.add_point(line.obj_.at(nearest_idx));
  if(nearest_idx == num_points-1) {
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

std::pair<DrivingCorridorPtr, bool> LocalMap::get_left_adjacent_corridor(const DrivingCorridorPtr driving_corridor) const {
  std::vector<LaneId> outer_lane_ids_new_corridor;

  for (auto const &lane_id : driving_corridor->get_lane_ids()) {
    LanePtr current_lane = map_interface_->get_lane(lane_id.second);
    std::pair<LaneId, bool> left_neighbor_id;

    if (current_lane->get_lane_position() > 0) {
      // Left neighbor is outer neighbor => the outer lane of the left adjacent
      // corridor is the outer neighbor of the outer neighbor of the current
      // lane
      std::pair<LanePtr, bool> left_neighbor =
        map_interface_->get_outer_neighbor(current_lane->get_id());
      left_neighbor = map_interface_->get_outer_neighbor(left_neighbor.first->get_id());

      if (left_neighbor.second) {
        left_neighbor_id.first = left_neighbor.first->get_id();
      }
      left_neighbor_id.second = left_neighbor.second;
    } else if (current_lane->get_lane_position() < 0) {
      // Left neighbor is inner neighbor => the outer lane of the left adjacent
      // corridor is the current lane
      left_neighbor_id = std::make_pair(current_lane->get_id(), true);
    } else {
      // Current lane is the plan view => outer lane of the left adjacent
      // corridor is the outer neighbor of the outer neighbor, but to which
      // side?
      std::pair<LanePtr, bool> outer_neighbor = map_interface_->get_outer_neighbor(current_lane->get_id());

      // Need to find out if the current corridor is left or right of the plan
      // view
      float signed_distance = geometry::signed_distance(current_lane->get_line(), geometry::get_point_at_s(driving_corridor->get_center(), 0.0f), 0);
      if (signed_distance * outer_neighbor.first->get_lane_position() < 0) {
        // This is the correct outer neighbor
        outer_neighbor = map_interface_->get_outer_neighbor(outer_neighbor.first->get_id());
      } else {
        // Need to obtain the other outer neighbor, if it exists
        outer_neighbor = map_interface_->get_outer_neighbor_but_not(current_lane->get_id(), outer_neighbor.first->get_id());
        if (outer_neighbor.second) {
          outer_neighbor = map_interface_->get_outer_neighbor(outer_neighbor.first->get_id());
        }
      }
      if (outer_neighbor.second) {
        left_neighbor_id.first = outer_neighbor.first->get_id();
      }
      left_neighbor_id.second = outer_neighbor.second;
    }

    if (!left_neighbor_id.second) {
      // Current lane has no left neighbor
      break;
    }

    if (!outer_lane_ids_new_corridor.empty()) {
      // Check if the new lane is a successor of the preceeding lane in the new
      // driving corridor
      LaneSequence successor_lanes_of_previous = map_interface_->get_successor_lanes(outer_lane_ids_new_corridor.back());
      if (std::find(successor_lanes_of_previous.begin(), successor_lanes_of_previous.end(), left_neighbor_id.first) == successor_lanes_of_previous.end()) {
        break;
      }
    }

    outer_lane_ids_new_corridor.push_back(left_neighbor_id.first);
  }

  if (outer_lane_ids_new_corridor.empty()) {
    return std::make_pair(nullptr, false);
  } else {
    DrivingCorridor driving_corridor = map_interface_->ComputeDrivingCorridorForRange(outer_lane_ids_new_corridor);
    return std::make_pair(DrivingCorridorPtr(new DrivingCorridor(driving_corridor)), true);
  }
}

std::pair<DrivingCorridorPtr, bool> LocalMap::get_right_adjacent_corridor(const DrivingCorridorPtr driving_corridor) const {
  std::vector<LaneId> outer_lane_ids_new_corridor;

  for (auto const &lane_id : driving_corridor->get_lane_ids()) {
    LanePtr current_lane = map_interface_->get_lane(lane_id.second);
    std::pair<LaneId, bool> right_neighbor_id;

    if (current_lane->get_lane_position() < 0) {
      // Right neighbor is outer neighbor => the outer lane of the right adjacent corridor is the outer neighbor of the outer neighbor of the current lane
      std::pair<LanePtr, bool> right_neighbor = map_interface_->get_outer_neighbor(current_lane->get_id());
      right_neighbor = map_interface_->get_outer_neighbor(right_neighbor.first->get_id());

      if (right_neighbor.second) {
        right_neighbor_id.first = right_neighbor.first->get_id();
      }
      right_neighbor_id.second = right_neighbor.second;
    } else if (current_lane->get_lane_position() > 0) {
      // Right neighbor is inner neighbor
      right_neighbor_id = std::make_pair(current_lane->get_id(), true);
    } else {
      // Current lane is the plan view => outer lane of the right adjacent corridor is the outer neighbor of the outer neighbor, but to which side?
      std::pair<LanePtr, bool> outer_neighbor = map_interface_->get_outer_neighbor(current_lane->get_id());

      // Need to find out if the current corridor is left or right of the plan view
      float signed_distance = geometry::signed_distance(current_lane->get_line(), geometry::get_point_at_s(driving_corridor->get_center(), 0.0f), 0);
      if (signed_distance * outer_neighbor.first->get_lane_position() > 0) {
        // This is the correct outer neighbor
        outer_neighbor = map_interface_->get_outer_neighbor(outer_neighbor.first->get_id());
      } else {
        // Need to obtain the other outer neighbor, if it exists
        outer_neighbor = map_interface_->get_outer_neighbor_but_not(current_lane->get_id(), outer_neighbor.first->get_id());
        if (outer_neighbor.second) {
          outer_neighbor = map_interface_->get_outer_neighbor(outer_neighbor.first->get_id());
        }
      }
      if (outer_neighbor.second) {
        right_neighbor_id.first = outer_neighbor.first->get_id();
      }
      right_neighbor_id.second = outer_neighbor.second;
    }

    if (!right_neighbor_id.second) {
      // Current lane has no right neighbor
      break;
    }

    if (!outer_lane_ids_new_corridor.empty()) {
      // Check if the new lane is a successor of the preceeding lane in the new driving corridor
      LaneSequence successor_lanes_of_previous = map_interface_->get_successor_lanes(outer_lane_ids_new_corridor.back());
      if (std::find(successor_lanes_of_previous.begin(), successor_lanes_of_previous.end(), right_neighbor_id.first) == successor_lanes_of_previous.end()) {
        break;
      }
    }

    outer_lane_ids_new_corridor.push_back(right_neighbor_id.first);
  }

  if (outer_lane_ids_new_corridor.empty()) {
    return std::make_pair(nullptr, false);
  } else {
    DrivingCorridor driving_corridor = map_interface_->ComputeDrivingCorridorForRange(outer_lane_ids_new_corridor);
    return std::make_pair(DrivingCorridorPtr(new DrivingCorridor(driving_corridor)), true);
  }
}

std::shared_ptr<LocalMap> LocalMap::Clone() const {
  LocalMapPtr new_local_map(new LocalMap(*this));
  return new_local_map;
}

}  // namespace map
}  // namespace world
}  // namespace modules
