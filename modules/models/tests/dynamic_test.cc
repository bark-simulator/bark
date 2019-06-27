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
#include "modules/models/dynamic/integration.hpp"

TEST(single_track_model, dynamic_test) {
  using namespace std;
  using namespace modules::geometry;
  using namespace modules::models::dynamic;

  State x(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  x << 0, 0, 0, 0, 5;

  Input u(2);
  u << 0, 0;

  DynamicModel *m;
  SingleTrackModel single_track_model;
  m = &single_track_model;

  float dt = 0.1;
  for (int i = 0; i < 10; i++) {
    x = euler_int(*m, x, u, dt);
    cout << x << endl;
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
