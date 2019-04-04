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


void LocalMap::concatenate_lines(const std::vector<LanePtr>& lanes,
                                 Line& line_of_corridor) {
  if (lanes.size() > 0) {
      line_of_corridor = lanes.at(0)->get_line();
      for (int i = 1; i < lanes.size(); i++) {
        if (lanes.at(i) != NULL) {
          line_of_corridor.concatenate_linestring(lanes.at(i)->get_line());
        }
      }
  }
}

// TODO(@hart): call this function only when the agent is created or the goal has changed
bool LocalMap::generate(Point2d point,
                        LaneId goal_lane_id,
                        double horizon) {
  std::vector<LanePtr> lanes;
  map_interface_->get_nearest_lanes(point, 1, lanes);
  LanePtr current_lane = lanes.at(0);

  std::pair< std::vector<LanePtr>, std::vector<LanePtr> > route =
    map_interface_->get_lane_boundary_horizon(current_lane->get_id(),
                                              goal_lane_id_);
  concatenate_lines(route.first, current_driving_corridor_.inner);
  concatenate_lines(route.second, current_driving_corridor_.outer);

  if (route.first[0] != NULL && route.second[0] != NULL) {
    current_driving_corridor_.center =
      geometry::calculate_center_line(current_driving_corridor_.inner,
                                      current_driving_corridor_.outer);
  }
}

}  // namespace map
}  // namespace world
}  // namespace modules
