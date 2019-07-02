// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include <limits>
#include <cmath>
#include <boost/geometry.hpp>
#include "modules/world/opendrive/lane_section.hpp"
#include "modules/world/opendrive/lane_section.hpp"
#include "modules/geometry/commons.hpp"

namespace modules {
namespace world {
namespace opendrive {

void LaneSection::add_lane(const LanePtr& lane)  {
    lanes_[lane->get_id()] = lane;
}

LanePtr LaneSection::get_lane_by_position(LanePosition pos) {

  LanePtr ret_lane_ptr = nullptr;

  for ( auto const& lane : lanes_ ) {
    if (pos == lane.second->get_lane_position()) {
      ret_lane_ptr = lane.second;
    }
  } 
  return ret_lane_ptr;
}


// TODO (@hart): replace dummy by real ray-check
LanePtr LaneSection::get_nearest_lane_on_n(double x, double y, double vx, double vy) {
  float x_new = x + vx;
  float y_new = y + vy;
  modules::geometry::Point2d new_point(x_new, y_new);
  float min_dist = 100000.0;
  LanePtr ret_lane_ptr = lanes_.begin()->second;
  
  for ( auto const& lane : lanes_ ) {
    float distance = modules::geometry::distance(lane.second->get_line(), new_point);
    if (distance < min_dist) {
      min_dist = distance;
      ret_lane_ptr = lane.second;
    }
  }
  
  return ret_lane_ptr;
}

LanePtr LaneSection::get_lane_with_offset(const models::dynamic::State& state, double angle_offset) {
  double x = state[models::dynamic::StateDefinition::X_POSITION];
  double y = state[models::dynamic::StateDefinition::Y_POSITION];
  double theta = state[models::dynamic::StateDefinition::THETA_POSITION];
  double theta_new = theta + angle_offset;

  LanePtr lane_ptr = get_nearest_lane_on_n(x, y, cos(theta_new), sin(theta_new));
  return lane_ptr;
}

}  // namespace opendrive
}  // namespace world
}  // namespace modules
