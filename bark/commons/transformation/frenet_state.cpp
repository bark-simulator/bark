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
  auto tangent_angle = mg::GetTangentAngleAtS(path, lon);
  auto norm_tangent_angle = mg::Norm0To2PI(tangent_angle);
  angle = mg::SignedAngleDiff(norm_tangent_angle,
                              state(StateDefinition::THETA_POSITION));
  auto direction_vector = pos - nearest_point;
  double diff = mg::SignedAngleDiff(
      norm_tangent_angle,
      atan2(bg::get<1>(direction_vector), bg::get<0>(direction_vector)));
  double sign = (diff > 0) ? -1 : ((diff < 0) ? 1 : 0);
  lat = lat_val * sign;

  // velocities
  const auto velocity = state[StateDefinition::VEL_POSITION];
  const auto orientation =
      mg::Norm0To2PI(state[StateDefinition::THETA_POSITION]);
  vlon = cos(std::abs(orientation - norm_tangent_angle)) * velocity;
  vlat = sin(std::abs(orientation - norm_tangent_angle)) * velocity;
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
  const auto angle =
      atan2(frenet_state.vlat, frenet_state.vlon) + line_angle;  // todo che

  const auto velocity = sqrt(frenet_state.vlon * frenet_state.vlon +
                             frenet_state.vlat * frenet_state.vlat);

  // build state
  State state(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  state(StateDefinition::TIME_POSITION) = 0.0f;
  state(StateDefinition::X_POSITION) = bg::get<0>(position);
  state(StateDefinition::Y_POSITION) = bg::get<1>(position);
  state(StateDefinition::THETA_POSITION) = angle;
  state(StateDefinition::VEL_POSITION) = velocity;
  return state;
}

}  // namespace transformation
}  // namespace commons
}  // namespace bark