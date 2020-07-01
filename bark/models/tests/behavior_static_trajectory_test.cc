// Copyright (c) 2020 Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "gtest/gtest.h"

#include "bark/commons/params/setter_params.hpp"
#include "bark/models/behavior/static_trajectory/behavior_static_trajectory.hpp"
#include "bark/world/observed_world.hpp"

using bark::models::behavior::BehaviorStaticTrajectory;
using bark::models::behavior::StateRowVector;
using bark::models::dynamic::Trajectory;
using bark::models::dynamic::StateDefinition;
using bark::models::dynamic::State;
using bark::world::World;
using bark::world::WorldPtr;
using bark::world::ObservedWorld;
using bark::commons::SetterParams;

TEST(behavior_static_trajectory_plan, plan) {
  Trajectory static_traj(4, static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  static_traj <<
  0, 0, 0, 0, 1,
  1, 1, 0, 0, 1,
  2, 2, 0, 0, 1,
  3, 3, 0, 0, 1;
  BehaviorStaticTrajectory model(nullptr, static_traj);
  auto params = std::make_shared<SetterParams>();
  WorldPtr world_ptr = std::make_shared<World>(params);
  ObservedWorld observed_world(world_ptr, 0);
  Trajectory traj;
  StateRowVector expected(static_cast<int>(StateDefinition::MIN_STATE_SIZE));

  // Return all
  traj = model.Plan(3, observed_world);
  ASSERT_EQ(4, traj.rows());

  traj = model.Plan(1, observed_world);
  ASSERT_EQ(2, traj.rows());
  EXPECT_EQ(static_traj.row(0), traj.row(0));
  EXPECT_EQ(static_traj.row(1), traj.row(1));

  // End interpolated
  traj = model.Plan(0.5, observed_world);
  ASSERT_EQ(2, traj.rows());
  EXPECT_EQ(static_traj.row(0), traj.row(0));
  expected << 0.5, 0.5, 0, 0, 1;
  EXPECT_EQ(expected, traj.row(1));

  // Start interpolated
  observed_world.Step(0.5);
  traj = model.Plan(0.5, observed_world);
  ASSERT_EQ(2, traj.rows());
  expected << 0.5, 0.5, 0, 0, 1;
  EXPECT_EQ(expected, traj.row(0));
  EXPECT_EQ(static_traj.row(1), traj.row(1));

  // Start and end interpolated
  traj = model.Plan(1, observed_world);
  ASSERT_EQ(3, traj.rows());
  expected << 0.5, 0.5, 0, 0, 1;
  EXPECT_EQ(expected, traj.row(0));
  EXPECT_EQ(static_traj.row(1), traj.row(1));
  expected << 1.5, 1.5, 0, 0, 1;
  EXPECT_EQ(expected, traj.row(2));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}