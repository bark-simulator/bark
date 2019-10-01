// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include <Eigen/Core>
#include "gtest/gtest.h"

#include "modules/geometry/polygon.hpp"
#include "modules/geometry/line.hpp"
#include "modules/geometry/commons.hpp"
#include "modules/models/dynamic/single_track.hpp"
#include "modules/models/dynamic/triple_integrator.hpp"
#include "modules/models/dynamic/integration.hpp"
#include "modules/commons/params/setter_params.hpp"
#include "modules/commons/params/default_params.hpp"


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
  DefaultParams* params = new DefaultParams();
  SingleTrackModel single_track_model(params);
  m = &single_track_model;

  float dt = 0.1;
  for (int i = 0; i < 10; i++) {
    x = euler_int(*m, x, u, dt);
    cout << x << endl;
  }
}

TEST(triple_integrator_model, dynamic_test) {
  using namespace std;
  using namespace modules::geometry;
  using namespace modules::models::dynamic;
  using namespace modules::commons;

  State x(15);
  x << 0, 0, 0, 0, 0, 0,  // time, x, y, theta, v, min_space
       0, 1, 0,  // x, vx, ax
       0, 1, 0,  // y, vy, ay
       0, 1, 0;  // z, vz, az

  Input u0(3);
  u0 << 0, 0, 0.;

  DynamicModel *m;
  DefaultParams* params = new DefaultParams();
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
