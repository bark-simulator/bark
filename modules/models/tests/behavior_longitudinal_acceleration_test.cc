// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Eigen/Core"
#include "gtest/gtest.h"

#include "make_test_world.hpp"
#include "modules/commons/params/default_params.hpp"
#include "modules/commons/params/setter_params.hpp"
#include "modules/geometry/commons.hpp"
#include "modules/geometry/line.hpp"
#include "modules/geometry/polygon.hpp"
#include "modules/models/behavior/longitudinal_acceleration/longitudinal_acceleration.hpp"
#include "modules/models/dynamic/single_track.hpp"
#include "modules/world/observed_world.hpp"

using namespace modules::models::dynamic;
using namespace modules::models::execution;
using namespace modules::commons;
using namespace modules::models::behavior;
using namespace modules::models::dynamic;
using namespace modules::world;
using namespace modules::geometry;
using namespace modules::models::tests;

// Acceleration
const double a = 10.0;

class DummyConstantAcceleration : public BehaviorLongitudinalAcceleration {
 public:
  using BehaviorLongitudinalAcceleration::BehaviorLongitudinalAcceleration;
  virtual double CalculateLongitudinalAcceleration(
      const ObservedWorld& observed_world) {
    return a;
  };

  virtual std::shared_ptr<BehaviorModel> Clone() const {
    std::shared_ptr<DummyConstantAcceleration> model_ptr =
        std::make_shared<DummyConstantAcceleration>(*this);
    return std::dynamic_pointer_cast<BehaviorModel>(model_ptr);
  };
};

TEST(behavior_constant_acceleration_plan, behavior_test) {
  SetterParams* params = new SetterParams();
  DummyConstantAcceleration behavior(params);

  const double dt = 1.0;

  // X Longitudinal with zero velocity
  ObservedWorld obs_world = make_test_observed_world(0, 0, 0, 0);

  Trajectory traj1 = behavior.Plan(1, obs_world);
  // s = 1/2 * a * t^2 + v_0*t
  double delta_s = 0.5 * a * dt * dt;
  EXPECT_EQ(1, traj1(traj1.rows() - 1, StateDefinition::TIME_POSITION));
  EXPECT_NEAR(
      delta_s + obs_world.get_ego_agent()->get_current_position().get<0>(),
      traj1(traj1.rows() - 1, StateDefinition::X_POSITION), .1e-4);

  // X Longitudinal with 5 m/s initial velocity
  const double v_0 = 5.0;
  auto obs_world1 = make_test_observed_world(0, 0, v_0, 0);
  Trajectory traj2 = behavior.Plan(1, obs_world1);
  // s = 1/2 * a * t^2 + v_0*t
  delta_s = 0.5 * a * dt * dt + v_0 * dt;
  EXPECT_EQ(1, traj1(traj2.rows() - 1, StateDefinition::TIME_POSITION));
  EXPECT_NEAR(
      delta_s + obs_world1.get_ego_agent()->get_current_position().get<0>(),
      traj2(traj2.rows() - 1, StateDefinition::X_POSITION), .1e-4);
}