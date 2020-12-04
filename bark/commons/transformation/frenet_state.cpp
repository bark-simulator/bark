// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/commons/transformation/frenet_state.hpp"

namespace bark {
namespace commons {
namespace transformation {

using bark::geometry::Line;
using bark::geometry::Point2d;
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
  auto norm_tangent_angle = angleRoad;
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

double TransformLatAccStreetToVehicle(double acc_lat_street, double acc_lon,
                                      double delta_time,
                                      const State& current_state,
                                      const FrenetState& current_frenet_state,
                                      const FrenetState& last_frenet_state) {
  double vel_lon = current_state(StateDefinition::VEL_POSITION);
  double theta = current_state(StateDefinition::THETA_POSITION);
  double theta_street = current_frenet_state.angleRoad;
  double delta_theta = mg::SignedAngleDiff(
      theta, theta_street);  // in fact, I think that is frenet_state.angle
  double route_heading_dot =
      (current_frenet_state.angleRoad - last_frenet_state.angleRoad) /
      delta_time;

  double acc_lat =
      (acc_lat_street - acc_lon * sin(delta_theta)) / cos(delta_theta) +
      vel_lon * route_heading_dot;
  VLOG(4) << "TransformLatAccStreetToVehicle() acc_lat_street="
          << acc_lat_street << " vel_lon=" << vel_lon << " acc_lon=" << acc_lon
          << " route_heading_dot=" << route_heading_dot
          << " delta_theta=" << delta_theta;
  return acc_lat;
}

}  // namespace transformation
}  // namespace commons
}  // namespace bark