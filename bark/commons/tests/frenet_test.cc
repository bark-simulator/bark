// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/commons/transformation/frenet.hpp"
#include "bark/commons/transformation/frenet_state.hpp"
#include "gtest/gtest.h"

using namespace bark::commons::transformation;
using namespace bark::geometry;
using namespace bark::models::dynamic;
using st = bark::models::dynamic::StateDefinition;

void test_state_two_way(const float x, const float y, const float theta,
                        const float v, const Line& line) {
  State state(static_cast<int>(st::MIN_STATE_SIZE));
  state << 0.0f, x, y, theta, v;

  FrenetState frenet_state(state, line);
  auto state_conv = FrenetStateToDynamicState(frenet_state, line);

  EXPECT_NEAR(state(st::X_POSITION), state_conv(st::X_POSITION), 0.001);
  EXPECT_NEAR(state(st::Y_POSITION), state_conv(st::Y_POSITION), 0.001);
  EXPECT_NEAR(state(st::VEL_POSITION), state_conv(st::VEL_POSITION), 0.001);
  EXPECT_NEAR(state(st::THETA_POSITION), state_conv(st::THETA_POSITION), 0.001);
}

void test_state_one_way(const float x, const float y, const float theta,
                        const float v, const Line& line, const float lat,
                        const float lon, const float vlat, const float vlon) {
  State state(static_cast<int>(st::MIN_STATE_SIZE));
  state << 0.0f, x, y, theta, v;

  FrenetState frenet_state(state, line);

  EXPECT_NEAR(frenet_state.lat, lat, 0.001);
  EXPECT_NEAR(frenet_state.lon, lon, 0.001);
  EXPECT_NEAR(frenet_state.vlat, vlat, 0.001);
  EXPECT_NEAR(frenet_state.vlon, vlon, 0.001);
}

TEST(frenet_state_one_way, slope_1) {
  // some line with three points from x=1 to x=10, y=0
  Line line;
  line.AddPoint(Point2d(0, 0));
  line.AddPoint(Point2d(1, 1));
  line.AddPoint(Point2d(2, 2));
  line.AddPoint(Point2d(4, 4));

  // state on path with orientation on path
  test_state_one_way(1, 0, atan2(1, 1), 5, line, -sqrt(0.5), sqrt(0.5), 0, 5);
}

TEST(frenet_state_one_way, slope_05) {
  // some line with three points from x=1 to x=10, y=0
  Line line;
  line.AddPoint(Point2d(-4, -2));
  line.AddPoint(Point2d(-2, -1));
  line.AddPoint(Point2d(2, 1));
  line.AddPoint(Point2d(4, 2));

  // state on path with orientation orthogonal on path
  test_state_one_way(1, 0.5, B_PI_2, 5.5, line, 0, sqrt(5 * 5 + 2.5 * 2.5),
                     cos(B_PI_2 - atan2(2, 1)) * 5.5,
                     sin(B_PI_2 - atan2(2, 1)) * 5.5);

  // state on left side of path with orientation on path
  test_state_two_way(1, 4, B_PI_2, 5, line);

  // state on right side of path with orientation on path
  test_state_two_way(-1, 5, B_PI_2, 5, line);
}

TEST(frenet_state_two_way, straight_line_right) {
  // some line with three points from x=1 to x=10, y=0
  Line line;
  line.AddPoint(Point2d(1, 0));
  line.AddPoint(Point2d(2, 0));
  line.AddPoint(Point2d(10, 0));

  // state on path with orientation on path
  test_state_two_way(3, 0, 0, 5, line);

  // state on left side of path with orientation on path
  test_state_two_way(2.5, 1, 0, 5, line);

  // state on right side of path with orientation on path
  test_state_two_way(8.2, -1, 0, 5, line);
}

TEST(frenet_state_two_way, straight_line_top) {
  // some line with three points from x=1 to x=10, y=0
  Line line;
  line.AddPoint(Point2d(0, 1));
  line.AddPoint(Point2d(0, 2));
  line.AddPoint(Point2d(0, 10));

  // state on path with orientation on path
  test_state_two_way(0, 3, B_PI_2, 5, line);

  // state on left side of path with orientation on path
  test_state_two_way(1, 4, B_PI_2, 5, line);

  // state on right side of path with orientation on path
  test_state_two_way(-1, 5, B_PI_2, 5, line);
}

TEST(frenet_state_two_way, straight_line_top_right) {
  // some line with three points from x=1 to x=10, y=0
  Line line;
  line.AddPoint(Point2d(-4, -2));
  line.AddPoint(Point2d(-2, -1));
  line.AddPoint(Point2d(2, 1));
  line.AddPoint(Point2d(4, 2));

  // state on path with orientation on path
  test_state_two_way(1, 0.5, B_PI_2, 5.5, line);

  // state on left side of path with orientation on path
  test_state_two_way(1, 4, B_PI_2, 5, line);

  // state on right side of path with orientation on path
  // test_state_two_way(-1, 5, B_PI_2, 5, line);
}