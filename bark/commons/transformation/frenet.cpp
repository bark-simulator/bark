// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/commons/transformation/frenet.hpp"

namespace bark {
namespace commons {
namespace transformation {

using bark::geometry::Line;
using bark::geometry::Point2d;
using bark::geometry::operator-;

FrenetPosition::FrenetPosition(const Point2d& position, const Line& path) {
  namespace bg = boost::geometry;

  // TODO(@hart): cover edge cases

  // First extract nearest point, extract longitudinal coordinate
  std::tuple<Point2d, double, uint> nearest =
      bark::geometry::GetNearestPointAndS(path, position);
  lon = std::get<1>(nearest);

  // calculate lateral coordinate value manually
  // to avoid getting the nearest point
  auto nearest_point = std::get<0>(nearest);
  auto x_diff = bg::get<0>(nearest_point) - bg::get<0>(position);
  auto y_diff = bg::get<1>(nearest_point) - bg::get<1>(position);
  double lat_val = sqrt(x_diff * x_diff + y_diff * y_diff);

  // calculate sign of lateral coordinate
  auto tangent_angle = bark::geometry::GetTangentAngleAtS(path, lon);
  auto direction_vector = position - nearest_point;
  double diff = bark::geometry::SignedAngleDiff(
      tangent_angle,
      atan2(bg::get<1>(direction_vector), bg::get<0>(direction_vector)));
  double sign = (diff > 0) ? -1 : ((diff < 0) ? 1 : 0);

  lat = lat_val * sign;
}

FrenetPosition FrenetPosition::operator+(const FrenetPosition& rhs) {
  return {lon + rhs.lon, lat + rhs.lat};
}

}  // namespace transformation
}  // namespace commons
}  // namespace bark