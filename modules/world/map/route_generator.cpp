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

    return map_interface_->get_driving_corridor(current_lane->get_id(), goal_lane_id_,
                           inner_line_, outer_line_, center_line_  );
  }
  else {
    return true;
  }
    
}

}  // namespace map
}  // namespace world
}  // namespace modules
