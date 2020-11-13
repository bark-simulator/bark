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
  auto tangent_angle = mg::GetTangentAngleAtS(path, lon);
  angle = mg::SignedAngleDiff(state(StateDefinition::THETA_POSITION),
                              tangent_angle);
  auto direction_vector = pos - nearest_point;
  auto norm_tangent_angle = mg::Norm0To2PI(tangent_angle);
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

auto ShapeExtensionAtTangentAngle(const double& tangent_angle, const Polygon& polygon) { 
  // Assumes equal extension to sides for polygon
  BARK_EXPECT_TRUE(std::abs(polygon.right_dist_ - polygon.left_dist_) < 0.01);
  const double polygon_side_extend = polygon.left_dist_;
  auto norm_tangent_angle = bark::geometry::Norm0ToPI(tangent_angle);
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
  const struct{double front_dist; double rear_dist; double left_dist; double right_dist;}
         shape_extension{front_dist, rear_dist, left_dist, right_dist};
  return shape_extension;
}


FrenetState FrenetStateDiffShapeExtension(const FrenetState& frenet_state1, const Polygon& polygon1,
                                        const FrenetState& frenet_state2, const Polygon& polygon2) {
    // Returns differences between frenet state and also considers projected shapes based on tangent angles
    // Conventions: 
    // - positive longitudinal difference if frenet_state2 is in "front" of frenet_state1
    // - positive lateral difference if frenet_state2 is in "left" of frenet_state1
    // - positive long and velocities diff if frenet state 2 is "faster" than frenet state1
    // - positive angle difference if frenet state2 is turned more left than frenet state 1

    BARK_EXPECT_TRUE(frenet_state1.Valid());
    BARK_EXPECT_TRUE(frenet_state2.Valid());

    auto shape_extend_at_tangent1 = ShapeExtensionAtTangentAngle(frenet_state1.angle, polygon1);
    auto shape_extend_at_tangent2 = ShapeExtensionAtTangentAngle(frenet_state2.angle, polygon2);

    FrenetState difference;
    if(frenet_state1.lon < frenet_state2.lon) {
      double diff_lon = frenet_state2.lon - shape_extend_at_tangent2.rear_dist -
                         (frenet_state1.lon + shape_extend_at_tangent1.front_dist);
      difference.lon = diff_lon > 0 ? diff_lon : 0;
    } else {
      double diff_lon = frenet_state2.lon + shape_extend_at_tangent2.front_dist -
                        (frenet_state1.lon - shape_extend_at_tangent1.rear_dist);
      difference.lon = diff_lon < 0 ? diff_lon : 0;
    }

    // the more negative the lateral coordinate is,
    // the more on the right side of the center line its state is
    if(frenet_state1.lat < frenet_state2.lat) {
      // lateral difference is positive
      double diff_lat = frenet_state2.lat - shape_extend_at_tangent2.right_dist -
                        (frenet_state1.lat + shape_extend_at_tangent1.left_dist);
      difference.lat = diff_lat > 0 ? diff_lat : 0;
    } else {
      // lateral difference is negative
      double diff_lat = frenet_state1.lat - shape_extend_at_tangent1.right_dist -
                        (frenet_state2.lat + shape_extend_at_tangent2.left_dist);
      difference.lat = diff_lat < 0 ? diff_lat : 0;
    }
    
    difference.vlat = frenet_state2.vlat - frenet_state1.vlat;
    difference.vlon = frenet_state2.vlon - frenet_state1.vlon;
    difference.angle = Norm0ToPI(Norm0To2PI(frenet_state2.angle - frenet_state1.angle));
    return difference;
}


}  // namespace transformation
}  // namespace commons
}  // namespace bark