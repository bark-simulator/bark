// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <Eigen/Core>
#include "gtest/gtest.h"

#include "modules/commons/params/default_params.hpp"
#include "modules/commons/params/setter_params.hpp"
#include "modules/geometry/commons.hpp"
#include "modules/geometry/line.hpp"
#include "modules/geometry/polygon.hpp"
#include "modules/models/dynamic/integration.hpp"
#include "modules/models/dynamic/single_track.hpp"
#include "modules/models/dynamic/triple_integrator.hpp"

TEST(single_track_model, dynamic_test) {
  using namespace std;
  using namespace modules::geometry;
  using namespace modules::models::dynamic;
  using namespace modules::commons;

  State x(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  x << 0, 0, 0, 0, 5;

  Input u(2);
  u << 0, 0;

  DynamicModel *m;
  auto params = std::make_shared<DefaultParams>();
  SingleTrackModel single_track_model(params);
  m = &single_track_model;

  float dt = 0.1;
  for (int i = 0; i < 10; i++) {
    x = euler_int(*m, x, u, dt);
    cout << x << endl;
  }
}

TEST(CalculateSteeringAngle, dynamic_test) {
  using namespace std;
  using namespace modules::geometry;
  using namespace modules::models::dynamic;
  using namespace modules::commons;

  const float dt = 1.0;
  auto a_lat = [dt](const State &x, const State &x1) {
    auto theta_dot = (x1(static_cast<int>(StateDefinition::THETA_POSITION)) -
                      x(static_cast<int>(StateDefinition::THETA_POSITION))) /
                     dt;
    return theta_dot * x(static_cast<int>(StateDefinition::VEL_POSITION));
  };

  auto params = std::make_shared<DefaultParams>();
  DynamicModel *m;
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
  using namespace modules::geometry;
  using namespace modules::models::dynamic;
  using namespace modules::commons;

  State x(15);
  x << 0, 0, 0, 0, 0, 0,  // time, x, y, theta, v, min_space
      0, 1, 0,            // x, vx, ax
      0, 1, 0,            // y, vy, ay
      0, 1, 0;            // z, vz, az

  Input u0(3);
  u0 << 0, 0, 0.;

  DynamicModel *m;
  auto params = std::make_shared<DefaultParams>();
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

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
