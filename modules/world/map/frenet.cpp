// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/world/map/frenet.hpp"

namespace modules {
namespace world {
namespace map {

using modules::geometry::Point2d;
using modules::geometry::Line;
using modules::geometry::operator-;

Frenet::Frenet(const Point2d& position, const Line& path) {
  namespace bg = boost::geometry;

  // First extract nearest point, extract longitudinal coordinate
  std::tuple<Point2d, double, uint> nearest = modules::geometry::get_nearest_point_and_s(path, position); 
  lon = std::get<1>(nearest);

  // calculate lateral coordinate value manually to avoid researching the nearest point
  auto nearest_point = std::get<0>(nearest);
  auto x_diff = bg::get<0>(nearest_point) - bg::get<0>(position);
  auto y_diff = bg::get<1>(nearest_point) - bg::get<1>(position);
  double lat_val = sqrt(x_diff*x_diff + y_diff*y_diff);

  // calculate sign of lateral coordinate
  auto tangent_angle = modules::geometry::get_tangent_angle_at_s(path, lon);
  auto direction_vector = position - nearest_point;
  double diff = modules::geometry::signed_angle_diff(tangent_angle , atan2(bg::get<1>(direction_vector), bg::get<0>(direction_vector)));
  double sign = (diff > 0) ? -1 : ((diff < 0) ? 1 : 0);

  lat = lat_val*sign;
}

}  // namespace map
}  // namespace world
}  // namespace modules