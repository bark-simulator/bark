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
      for (int i = 1; i < lanes.size(); i++) {
        if (lanes.at(i) != NULL) {
          lane_ids.push_back(std::pair<int, LaneId>(line_of_corridor.size(),
                                                    lanes.at(i)->get_id()));
          line_of_corridor.ConcatenateLinestring(lanes.at(i)->get_line());
        }
      }
  }
}

LaneId LocalMap::GoalLaneIdFromGoalDefinition(const GoalDefinition& goal_definition) {
  const modules::geometry::Point2d& goal_center = goal_definition.get_shape().center_;
  std::vector<opendrive::LanePtr>& nearest_lanes;

  if(map_interface_->FindNearestLanes(goal_center, 1, nearest_lanes)) {
      return nearest_lanes[0]->get_id();
  }
  printf("No matching lane for goal definition found. Defaulting to LaneId 0.")
  return LaneId(0);
}

bool LocalMap::Generate(Point2d point, double horizon) {
  goal_lane_id_ = GoalLaneIdFromGoalDefinition(goal_definition_);

  std::vector<LanePtr> lanes;
  map_interface_->FindNearestLanes(point, 1, lanes);
  LanePtr current_lane = lanes.at(0);

  std::pair< std::vector<LanePtr>, std::vector<LanePtr> > route =
    map_interface_->ComputeLaneBoundariesHorizon(current_lane->get_id(),
                                                 goal_lane_id_);

  std::vector< std::pair<int, LaneId> > dummy;
  ConcatenateLines(route.first,
                   driving_corridor_.inner,
                   driving_corridor_.lane_ids_);
  ConcatenateLines(route.second,
                   driving_corridor_.outer,
                   dummy);
  if (route.first[0] != NULL && route.second[0] != NULL) {
    driving_corridor_.center =
      ComputeCenterLine(driving_corridor_.inner,
                        driving_corridor_.outer);
  }
  driving_corridor_.computed = true;

  return true;
}

DrivingCorridor LocalMap::ComputeDrivingCorridor(std::vector<LaneId> lane_ids) {
  std::pair< std::vector<LanePtr>, std::vector<LanePtr> > route =
    map_interface_->ComputeLaneBoundaries(lane_ids);
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
  int nearest_idx = FindNearestIdx(line, p);
  int max_idx = line.obj_.size();
  double s = 0.0;
  for (int idx = nearest_idx; nearest_idx < max_idx - 1; idx++) {
    // TODO(@hart): in case of non circular routes.. criteria to abort?
    int idx_ = idx % max_idx;  // in case of circle
    int idx_next = (idx+1) % max_idx;  // in case of circle
    double d = distance(line.obj_.at(idx_), line.obj_.at(idx_next));
    new_line.add_point(line.obj_.at(idx_next));
    s += d;
    if ( s > horizon )
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
