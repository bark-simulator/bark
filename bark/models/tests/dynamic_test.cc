// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <Eigen/Core>
#include "gtest/gtest.h"

#include "bark/commons/params/setter_params.hpp"
#include "bark/geometry/commons.hpp"
#include "bark/geometry/line.hpp"
#include "bark/geometry/polygon.hpp"
#include "bark/models/dynamic/integration.hpp"
#include "bark/models/dynamic/single_track.hpp"
#include "bark/models/dynamic/triple_integrator.hpp"

using namespace std;
using namespace bark::geometry;
using namespace bark::models::dynamic;
using namespace bark::commons;

TEST(single_track_model, dynamic_test) {
  State x(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  x << 0, 0, 0, 0, 5;

  Input u(2);
  u << 0, 0;

  DynamicModel* m;
  auto params = std::make_shared<SetterParams>();
  SingleTrackModel single_track_model(params);
  m = &single_track_model;

  float dt = 0.1;
  for (int i = 0; i < 10; i++) {
    x = euler_int(*m, x, u, dt);
    cout << x << endl;
  }
}

TEST(valid_state_test, dynamic_test) {
  State x1(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  x1 << 50.0f, 0.0f, 0.0f, M_PI / 2.0f, 50.0f;
  EXPECT_TRUE(IsValid(x1));

  // nan
  State x2(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  x2 << 50.0f, 0.0f, 1 / 0.0f, M_PI / 2.0f, 50.0f;
  EXPECT_FALSE(IsValid(x2));

  // inf
  State x3(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  x3 << 50.0f, 0.0f / 0.0f, 1 / 0.0f, M_PI / 2.0f, 50.0f;
  EXPECT_FALSE(IsValid(x3));

  // nan and inf
  State x4(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  x4 << 50.0f, 0.0 / 0.0f, 1 / 0.0f, M_PI / 2.0f, 50.0f;
  EXPECT_FALSE(IsValid(x4));
}

TEST(valid_trajectory_test, dynamic_test) {
  Trajectory traj1(4, static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  traj1 << 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 2, 2, 0, 0, 1, 3, 3, 0, 0, 1;
  EXPECT_TRUE(IsValid(traj1));

  // inf
  Trajectory traj2(4, static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  traj2 << 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 2, 2, 1 / 0.0f, 0, 1, 3, 3, 0, 0, 1;
  EXPECT_FALSE(IsValid(traj2));

  // nan
  Trajectory traj3(4, static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  traj3 << 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 2, 2, 0.0 / 0.0f, 0, 1, 3, 3, 0, 0, 1;
  EXPECT_FALSE(IsValid(traj3));

  // both
  Trajectory traj4(4, static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  traj4 << 0, 1 / 0.0f, 0, 0, 1, 1, 1, 0, 0, 1, 2, 2, 0, 0, 1, 3, 3, 0, 0,
      0.0 / 0.0f;
  EXPECT_FALSE(IsValid(traj4));
}

TEST(CalculateSteeringAngle, dynamic_test) {
  using namespace std;
  using namespace bark::geometry;
  using namespace bark::models::dynamic;
  using namespace bark::commons;

  const float dt = 1.0;
  auto a_lat = [dt](const State& x, const State& x1) {
    auto theta_dot = (x1(static_cast<int>(StateDefinition::THETA_POSITION)) -
                      x(static_cast<int>(StateDefinition::THETA_POSITION))) /
                     dt;
    return theta_dot * x(static_cast<int>(StateDefinition::VEL_POSITION));
  };

  auto params = std::make_shared<SetterParams>();
  DynamicModel* m;
  SingleTrackModelPtr single_track_model =
      std::make_shared<SingleTrackModel>(params);
  m = single_track_model.get();
  const float a_lat_max = single_track_model->GetLatAccelerationMax();
  const float delta_max = single_track_model->GetSteeringAngleMax();

  State x(static_cast<int>(StateDefinition::MIN_STATE_SIZE));

  Input u(2);

  Point2d point_1(0.0, 0.0);
  Point2d point_2(0.0, 10.0);

  Line_t<Point2d> line;

  line.AddPoint(point_1);
  line.AddPoint(point_2);

  // Parallel to line, high crosstrack error, high speed
  x << 50.0f, 0.0f, 0.0f, M_PI / 2.0f, 50.0f;
  auto delta = CalculateSteeringAngle(single_track_model, x, line, 1.0);
  u << 0.0f, delta;
  auto x1 = euler_int(*m, x, u, dt);

  EXPECT_LE(std::abs(a_lat(x, x1)), a_lat_max);
  EXPECT_LE(std::abs(delta), delta_max);
  EXPECT_LT(x1(static_cast<int>(StateDefinition::X_POSITION)),
            x(static_cast<int>(StateDefinition::X_POSITION)));

  // Parallel to line, high crosstrack error, low speed
  x << 50.0f, 0.0f, 0.0f, M_PI / 2.0f, 0.1f;
  delta = CalculateSteeringAngle(single_track_model, x, line, 1.0);
  u << 0.0f, delta;
  x1 = euler_int(*m, x, u, dt);

  EXPECT_LE(std::abs(a_lat(x, x1)), a_lat_max);
  EXPECT_LE(std::abs(delta), delta_max);
  EXPECT_LT(x1(static_cast<int>(StateDefinition::X_POSITION)),
            x(static_cast<int>(StateDefinition::X_POSITION)));
}

TEST(triple_integrator_model, dynamic_test) {
  using namespace std;
  using namespace bark::geometry;
  using namespace bark::models::dynamic;
  using namespace bark::commons;

  State x(15);
  x << 0, 0, 0, 0, 0, 0,  // time, x, y, theta, v, min_space
      0, 1, 0,            // x, vx, ax
      0, 1, 0,            // y, vy, ay
      0, 1, 0;            // z, vz, az

  Input u0(3);
  u0 << 0, 0, 0.;

  DynamicModel* m;
  auto params = std::make_shared<SetterParams>();
  TripleIntegratorModel triple_int_model(params);
  m = &triple_int_model;

  float dt = 0.1;
  for (int i = 0; i < 10; i++) {
    x = euler_int(*m, x, u0, dt);
    cout << x << endl << endl;
  }
  // TODO(@hart): assert state
  u0 << 1., 1., 1.;
  for (int i = 0; i < 10; i++) {
    x = euler_int(*m, x, u0, dt);
    cout << x << endl << endl;
  }
  // TODO(@hart): assert state
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
