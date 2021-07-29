// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/commons/transformation/frenet_state.hpp"
#include "bark/geometry/angle.hpp"

namespace bark {
namespace commons {
namespace transformation {

using namespace bark::geometry;
using bark::geometry::operator-;
using bark::geometry::operator*;
using bark::geometry::operator+;
using bark::models::dynamic::State;
using bark::models::dynamic::StateDefinition;


namespace bg = boost::geometry;
namespace mg = bark::geometry;

FrenetState::FrenetState(const State& state, const Line& path) {
  // First extract nearest point, extract longitudinal coordinate
  Point2d pos(state(StateDefinition::X_POSITION),
              state(StateDefinition::Y_POSITION));
  std::tuple<Point2d, double, uint> nearest =
      mg::GetNearestPointAndS(path, pos);
  lon = std::get<1>(nearest);

  // calculate lateral coordinate value manually to avoid researching the
  // nearest point
  auto nearest_point = std::get<0>(nearest);
  auto x_diff = bg::get<0>(nearest_point) - bg::get<0>(pos);
  auto y_diff = bg::get<1>(nearest_point) - bg::get<1>(pos);
  double lat_val = sqrt(x_diff * x_diff + y_diff * y_diff);

  // calculate sign of lateral coordinate
  const auto orientation = state[StateDefinition::THETA_POSITION];
  angleRoad = mg::GetTangentAngleAtS(path, lon);
  angle = mg::SignedAngleDiff(angleRoad, orientation);
  auto direction_vector = pos - nearest_point;
  double diff = mg::SignedAngleDiff(
      angleRoad,
      atan2(bg::get<1>(direction_vector), bg::get<0>(direction_vector)));
  double sign = (diff > 0) ? -1 : ((diff < 0) ? 1 : 0);
  VLOG(5) << "Orientation: " << orientation << ", Frenet Angle: " << angle
          << ", Sign: " << sign;
  lat = lat_val * sign;

  // velocities
  const auto velocity = state[StateDefinition::VEL_POSITION];
  vlon = cos(mg::SignedAngleDiff(orientation, angleRoad)) * velocity;
  vlat = sin(mg::SignedAngleDiff(orientation, angleRoad)) * velocity;
}

State FrenetStateToDynamicState(const FrenetState& frenet_state,
                                const Line& path) {
  // calculate position
  const auto line_point = mg::GetPointAtS(path, frenet_state.lon);
  const auto line_angle = mg::GetTangentAngleAtS(path, frenet_state.lon);
  const auto normal = mg::GetNormalAtS(path, frenet_state.lon);
  const auto position = line_point + normal * frenet_state.lat;

  // calculate angle from frenet velocities
  //  std::cout << frenet_state.vlat << ", " << frenet_state.vlon << std::endl;
  const auto angle = mg::NormToPI(atan2(frenet_state.vlat, frenet_state.vlon) +
                                  line_angle);  // todo che

  const auto velocity = sqrt(frenet_state.vlon * frenet_state.vlon +
                             frenet_state.vlat * frenet_state.vlat);

  // build state
  State state(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  state(StateDefinition::TIME_POSITION) = 0.0;
  state(StateDefinition::X_POSITION) = bg::get<0>(position);
  state(StateDefinition::Y_POSITION) = bg::get<1>(position);
  state(StateDefinition::THETA_POSITION) = angle;
  state(StateDefinition::VEL_POSITION) = velocity;
  return state;
}

/**
 * @brief Calculate extension of a shape oriented with tangent_angle along a straight line.
 *        Assumes equal extension of original shape to left and right side
 * 
 * @param tangent_angle 
 * @param polygon 
 * @return A struct containing rear_dist, front_dist, left_dist, side_dist of rotated shape
 */
ShapeExtension ShapeExtensionAtTangentAngle(const double& tangent_angle, const Polygon& polygon) { 
  // Assumes equal extension to sides for polygon
  BARK_EXPECT_TRUE(std::abs(polygon.right_dist_ - polygon.left_dist_) < 0.01);
  const double polygon_side_extend = polygon.left_dist_;
  auto norm_tangent_angle = bark::geometry::NormToPI(tangent_angle);
  auto positive_angle = std::abs(tangent_angle) < B_PI_2 ? std::abs(norm_tangent_angle) : std::abs(std::abs(norm_tangent_angle) - B_PI);
  double front_dist = cos(positive_angle)*polygon.front_dist_ + sin(positive_angle)*polygon_side_extend;
  double rear_dist = cos(positive_angle)*polygon.rear_dist_ + sin(positive_angle)*polygon_side_extend;
  double left_dist = sin(positive_angle)*polygon.front_dist_ + cos(positive_angle)*polygon_side_extend;
  double right_dist = sin(positive_angle)*polygon.rear_dist_ + cos(positive_angle)*polygon_side_extend;

  if(tangent_angle > B_PI_2) {
    std::swap(front_dist, rear_dist);
  } else if(tangent_angle < -B_PI_2) {
    std::swap(front_dist, rear_dist);
    std::swap(left_dist, right_dist);
  } else {
    std::swap(left_dist, right_dist);
  }
  ShapeExtension shape_extension{front_dist, rear_dist, left_dist, right_dist};
  return shape_extension;
}

/**
 * @brief Construct a new Frenet State Difference. The state difference also
 *         considers the shape sand their relative orientations. If there is an overlap of shapes
 *         a distance without shape information is calculated.
 * 
 * @param frenet_from 
 * @param polygon_from 
 * @param frenet_to 
 * @param polygon_to 
 */
FrenetStateDifference::FrenetStateDifference(const FrenetState& frenet_from, const bark::geometry::Polygon& polygon_from,
                                        const FrenetState& frenet_to, const bark::geometry::Polygon& polygon_to) :
                                        FrenetState(),
                                        from(frenet_from),
                                        to(frenet_to) {
  BARK_EXPECT_TRUE(from.Valid());
  BARK_EXPECT_TRUE(to.Valid());

  auto shape_extend_at_tangent1 = ShapeExtensionAtTangentAngle(from.angle, polygon_from);
  auto shape_extend_at_tangent2 = ShapeExtensionAtTangentAngle(to.angle, polygon_to);

  if(from.lon <= to.lon) {
    double diff_lon = to.lon - shape_extend_at_tangent2.rear_dist -
                        (from.lon + shape_extend_at_tangent1.front_dist);
    lon_zeroed = diff_lon <= 0;
    lon = diff_lon > 0 ? diff_lon : to.lon - from.lon;
  } else {
    double diff_lon = to.lon + shape_extend_at_tangent2.front_dist -
                      (from.lon - shape_extend_at_tangent1.rear_dist);
    lon_zeroed = diff_lon >= 0;
    lon = diff_lon < 0 ? diff_lon : to.lon - from.lon;
  }

  // the more negative the lateral coordinate is,
  // the more on the right side of the center line its state is
  if(from.lat <= to.lat) {
    // lateral difference is positive
    double diff_lat = to.lat - shape_extend_at_tangent2.right_dist -
                      (from.lat + shape_extend_at_tangent1.left_dist);
    // if shape consideration leads to negative distance use only pure lateral difference
    lat_zeroed = diff_lat <= 0;
    lat = diff_lat > 0 ? diff_lat : to.lat - from.lat;
  } else {
    // lateral difference is negative
    double diff_lat = to.lat + shape_extend_at_tangent1.right_dist -
                      (from.lat - shape_extend_at_tangent2.right_dist);
    lat_zeroed = diff_lat >= 0;
    lat = diff_lat < 0 ? diff_lat : to.lat - from.lat;
  }
  
  vlat = to.vlat - from.vlat;
  vlon = to.vlon - from.vlon;
  angle = NormToPI(Norm0To2PI(to.angle - from.angle));
}


/**
 * @brief Transforms Lateral Acceleration from Streetwise to Vehicle Coordinate
 * System
 *
 * @param acc_lat_street ... lateral acceleration (street corrdinate system)
 * @param acc_lon ... longitudinal acceleration
 * @param delta_time ... time increment from t_i-1 to t_i
 * @param current_state ... state of vehicle at t_i
 * @param current_frenet_state ... frenet state of vehicle t_i
 * @param last_frenet_state ... frenet state at t_i-1
 * @return double lateral acceleration (vehicle coordinate system)
 */
double LatAccStreetToVehicleCs(double acc_lat_street, double acc_lon,
                               double delta_time, const State& current_state,
                               const FrenetState& current_frenet_state,
                               const FrenetState& last_frenet_state) {
  double vel_lon = current_state(StateDefinition::VEL_POSITION);
  double theta = current_state(StateDefinition::THETA_POSITION);
  double theta_street = current_frenet_state.angleRoad;
  double delta_theta = mg::SignedAngleDiff(theta, theta_street);
  double route_heading_dot = mg::SignedAngleDiff(current_frenet_state.angleRoad,
                                                 last_frenet_state.angleRoad) /
                             delta_time;

  double acc_lat =
      (acc_lat_street - acc_lon * sin(delta_theta)) / cos(delta_theta) +
      vel_lon * route_heading_dot;
  VLOG(4) << "LatAccStreetToVehicleCs() acc_lat_street=" << acc_lat_street
          << " vel_lon=" << vel_lon << " acc_lon=" << acc_lon
          << " route_heading_dot=" << route_heading_dot
          << " delta_theta=" << delta_theta
          << " acc_lat=" << acc_lat
          << " sin=" << sin(delta_theta)
          << " cos=" << cos(delta_theta)
          << " 1st term=" << (acc_lat_street - acc_lon * sin(delta_theta)) / cos(delta_theta)
          << " 2nd term=" << vel_lon * route_heading_dot;
  return acc_lat;
}

}  // namespace transformation
}  // namespace commons
}  // namespace bark