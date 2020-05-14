// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Eigen/Core"
#include "gtest/gtest.h"

#include "bark/world/tests/make_test_world.hpp"
#include "bark/commons/params/default_params.hpp"
#include "bark/commons/params/setter_params.hpp"
#include "bark/geometry/commons.hpp"
#include "bark/geometry/line.hpp"
#include "bark/geometry/polygon.hpp"
#include "bark/models/behavior/longitudinal_acceleration/longitudinal_acceleration.hpp"
#include "bark/models/dynamic/single_track.hpp"
#include "bark/world/observed_world.hpp"

using namespace modules::models::dynamic;
using namespace modules::models::execution;
using namespace modules::commons;
using namespace modules::models::behavior;
using namespace modules::models::dynamic;
using namespace modules::world;
using namespace modules::geometry;
using namespace modules::world::tests;

// Acceleration
const double a = 10.0;

class DummyConstantAcceleration : public BehaviorLongitudinalAcceleration {
 public:
  using BehaviorLongitudinalAcceleration::BehaviorLongitudinalAcceleration;
  virtual double CalculateLongitudinalAcceleration(
      const ObservedWorld& observed_world) {
    return a;
  }

  virtual std::shared_ptr<BehaviorModel> Clone() const {
    std::shared_ptr<DummyConstantAcceleration> model_ptr =
        std::make_shared<DummyConstantAcceleration>(*this);
    return std::dynamic_pointer_cast<BehaviorModel>(model_ptr);
  }
};

TEST(behavior_constant_acceleration_plan, behavior_test_zero_velocity) {
  auto params = std::make_shared<SetterParams>();
  DummyConstantAcceleration behavior(params);

  float dt = 1.0;
  float vel0 = 0;
  // test only in x-direction
  ObservedWorld obs_world = make_test_observed_world(0, 0, vel0, 0);

  Trajectory traj1 = behavior.Plan(dt, obs_world);
  auto last_idx = traj1.rows() - 1;
  EXPECT_EQ(dt, traj1(last_idx, StateDefinition::TIME_POSITION));

  auto x = obs_world.GetEgoAgent()
               ->GetCurrentState()[StateDefinition::X_POSITION];
  double dx = vel0 * dt + 0.5 * a * dt * dt;
  EXPECT_FLOAT_EQ(dx + x, traj1(last_idx, StateDefinition::X_POSITION));
}

TEST(behavior_constant_acceleration_plan, behavior_test_non_zero_velocity) {
  auto params = std::make_shared<SetterParams>();
  DummyConstantAcceleration behavior(params);

  float dt = 1.0;
  float vel0 = 5.0;

  // test only in x-direction
  ObservedWorld obs_world = make_test_observed_world(0, 0, vel0, 0);

  Trajectory traj1 = behavior.Plan(dt, obs_world);
  auto last_idx = traj1.rows() - 1;
  EXPECT_EQ(1, traj1(traj1.rows() - 1, StateDefinition::TIME_POSITION));

  auto x = obs_world.GetEgoAgent()
               ->GetCurrentState()[StateDefinition::X_POSITION];
  double dx = vel0 * dt + 0.5 * a * dt * dt;
  EXPECT_FLOAT_EQ(dx + x, traj1(traj1.rows() - 1, StateDefinition::X_POSITION));
}