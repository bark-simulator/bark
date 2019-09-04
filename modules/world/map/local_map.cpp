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

void LocalMap::ConcatenateLines(const std::vector<LanePtr>& lanes,
                                Line& line_of_corridor,
                                std::vector<std::pair<int, LaneId>>& lane_ids) {
  if (lanes.size() > 0) {
      line_of_corridor = lanes.at(0)->get_line();
      lane_ids.push_back(std::pair<int, LaneId>(0, lanes.at(0)->get_id()));
      for (uint i = 1; i < lanes.size(); i++) {
        if (lanes.at(i) != NULL) {
          lane_ids.push_back(std::pair<int, LaneId>(line_of_corridor.size(),
                                                    lanes.at(i)->get_id()));
          line_of_corridor.ConcatenateLinestring(lanes.at(i)->get_line());
        }
      }
  }
}

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

bool LocalMap::Generate(Point2d point, double horizon) {
  if(map_interface_ == nullptr) {
    return false;
  }

  auto goal_definition_polygon = std::dynamic_pointer_cast<GoalDefinitionPolygon>(goal_definition_);
  if (!goal_definition_polygon) {
    return false; //< todo: handle this better
  }

  goal_lane_id_ = GoalLaneIdFromGoalDefinitionPolygon(*goal_definition_polygon);

  std::vector<LanePtr> lanes;
  map_interface_->FindNearestLanes(point, 1, lanes);
  LanePtr current_lane = lanes.at(0);

  std::pair< std::vector<LanePtr>, std::vector<LanePtr> > route =
    map_interface_->ComputeLaneBoundariesHorizon(current_lane->get_id(),
                                                 goal_lane_id_);


  if (route.first.size() != 0 && route.second.size() != 0) {
    std::vector< std::pair<int, LaneId> > dummy;
    ConcatenateLines(route.first,
                    driving_corridor_.inner,
                    driving_corridor_.lane_ids_);
    ConcatenateLines(route.second,
                    driving_corridor_.outer,
                    dummy);
                    
    driving_corridor_.center = ComputeCenterLine(driving_corridor_.inner,
                                                 driving_corridor_.outer);
    driving_corridor_.computed = true;
  }

  return true;
}

DrivingCorridor LocalMap::ComputeDrivingCorridor(std::vector<LaneId> lane_ids) {
  std::pair< std::vector<LanePtr>, std::vector<LanePtr> > route =
    map_interface_->get_roadgraph()->ComputeRouteBoundaries(lane_ids);
  DrivingCorridor dc;
  std::vector< std::pair<int, LaneId> > dummy;
  ConcatenateLines(route.first, dc.inner, dc.lane_ids_);
  ConcatenateLines(route.second, dc.outer, dummy);
  if (route.first[0] != NULL && route.second[0] != NULL) {
    dc.center = ComputeCenterLine(dc.inner, dc.outer);
  }
  dc.computed = true;
  return dc;
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

}  // namespace map
}  // namespace world
}  // namespace modules
