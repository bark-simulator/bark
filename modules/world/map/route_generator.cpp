// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <utility>
#include "modules/world/map/route_generator.hpp"
#include "modules/world/map/map_interface.hpp"

namespace modules {
namespace world {
namespace map {

using modules::world::opendrive::LanePtr;
bool RouteGenerator::generate(Point2d point,
                              LaneId goal_lane_id,
                              double horizon) {

  // TODO(@hart): a flag if route should be updated would be better
  if (!computed_) {
    std::vector<LanePtr> lanes;
    map_interface_->get_nearest_lanes(point, 1, lanes);
    LanePtr current_lane = lanes.at(0);

    std::cout << "Nearest lane_id: " << current_lane->get_id() << std::endl;

    // TODO: with parameter --> length
    std::pair< std::vector<LanePtr>, std::vector<LanePtr> > route =
      map_interface_->get_lane_boundary_horizon(current_lane->get_id(),
                                                goal_lane_id_);


    if (route.first[0]) {
      inner_line_ = route.first[0]->get_line();
      // inner lane
      for (int i = 1; i < route.first.size(); i++) {
        if (route.first[i] != NULL) {
          inner_line_.concatenate_linestring(route.first[i]->get_line());
        }
      }
    }
    if (route.second[0]) {
      outer_line_ = route.second[0]->get_line();
      // inner lane
      for (int i = 1; i < route.second.size(); i++) {
        if (route.second[i] != NULL) {
          outer_line_.concatenate_linestring(route.second[i]->get_line());
        }
      }
    }

    if (route.first[0] != NULL && route.second[0] != NULL) {
      center_line_ = calculate_center_line();
      computed_ = true;
    } else {
      return false;
    }
  }
  return true;
}

Line RouteGenerator::calculate_center_line(double horizon) {
  // select line with more points for looping to avoid resolution reduction
  Line line_more_points = outer_line_;
  Line line_less_points = inner_line_;
  if ( inner_line_.obj_.size() > outer_line_.obj_.size() ) {
    line_more_points = inner_line_;
    line_less_points = outer_line_;
  }
  // TODO(@hart): use horizon
  for ( Point2d& point_loop : line_more_points.obj_ ) {
    Point2d nearest_point_other = geometry::get_nearest_point(line_less_points,
                                                              point_loop);
    geometry::Point2d middle_point = (point_loop + nearest_point_other) / 2;
    center_line_.add_point(middle_point);
  }

  return center_line_;
}

}  // namespace map
}  // namespace world
}  // namespace modules
