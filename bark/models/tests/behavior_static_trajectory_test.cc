// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "boost/variant.hpp"
#include "gtest/gtest.h"

#include "bark/models/behavior/static_trajectory/behavior_static_trajectory.hpp"
#include "bark/world/observed_world.hpp"
#include "bark/world/tests/make_test_world.hpp"

using bark::geometry::Point2d;
using bark::geometry::Polygon;
using bark::geometry::Pose;
using bark::models::behavior::BehaviorStaticTrajectory;
using bark::models::behavior::LonLatAction;
using bark::models::behavior::StateRowVector;
using bark::models::dynamic::State;
using bark::models::dynamic::StateDefinition;
using bark::models::dynamic::Trajectory;
using bark::world::Agent;
using bark::world::ObservedWorld;
using bark::world::World;
using bark::world::WorldPtr;
using bark::world::goal_definition::GoalDefinitionPolygon;

TEST(behavior_static_trajectory_plan, plan) {
  Trajectory static_traj(4, static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  static_traj << 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 2, 2, 0, 0, 1, 3, 3, 0, 0, 1;
  BehaviorStaticTrajectory model(nullptr, static_traj);
  auto observed_world =
      bark::world::tests::make_test_observed_world(0, 0, 0, 0);
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

TEST(behavior_static_trajectory_plan, calculate_action) {
  // two lane road along x= 0 , 200, lane1 -1.75, lan2 -3.0
  Polygon polygon(Pose(0, 0, 0),
                  std::vector<Point2d>{Point2d(-10, 0.2), Point2d(10, 0.2),
                                       Point2d(10, -0.2), Point2d(-10, -0.2),
                                       Point2d(-10, 0.2)});
  std::shared_ptr<Polygon> goal_polygon(
      std::dynamic_pointer_cast<Polygon>(polygon.Translate(
          Point2d(50, -1.75))));  // < move the goal polygon into the driving
                                  // corridor in front of the ego vehicle
  auto goal_def = std::make_shared<GoalDefinitionPolygon>(polygon);
  auto observed_world =
      bark::world::tests::make_test_observed_world(0, 0, 5, 0, goal_def);
  Trajectory traj;
  StateRowVector expected(static_cast<int>(StateDefinition::MIN_STATE_SIZE));

  // zero accelerations
  Trajectory static_traj1(2, static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  static_traj1 << 0, 0, -1.75, 0, 1, 1, 1, -1.75, 0, 1;
  auto action = BehaviorStaticTrajectory::CalculateAction(1.0, observed_world,
                                                          static_traj1);
  LonLatAction lon_lat_action = boost::get<LonLatAction>(action);
  EXPECT_NEAR(lon_lat_action.acc_lon, 0.0, 0.001);
  EXPECT_NEAR(lon_lat_action.acc_lat, 0.0, 0.001);

  // non-zero lon, zero lat accelerations
  Trajectory static_traj2(3, static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  static_traj2 << 0, 0, -1.75, 0, 0, 1, 1, -1.75, 0, 1, 2, 1, -1.75, 0, 1;
  LOG(INFO) << static_traj2.row(0);
  action = BehaviorStaticTrajectory::CalculateAction(2.0, observed_world,
                                                     static_traj2);
  lon_lat_action = boost::get<LonLatAction>(action);
  EXPECT_NEAR(lon_lat_action.acc_lon, 0.5, 0.001);
  EXPECT_NEAR(lon_lat_action.acc_lat, 0.0, 0.001);

  // zero lon, non-zero lat accelerations
  Trajectory static_traj3(3, static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  static_traj3 << 0, 0, -1.75, 3.14 / 2.0, 0, 1, 1, -1.75, 3.14 / 2.0, 1, 2, 1,
      -3, 3.14 / 2.0, 1;
  action = BehaviorStaticTrajectory::CalculateAction(2.0, observed_world,
                                                     static_traj3);
  lon_lat_action = boost::get<LonLatAction>(action);
  EXPECT_NEAR(lon_lat_action.acc_lon, 0.0, 0.001);
  EXPECT_NEAR(lon_lat_action.acc_lat, 1 / 2.0, 0.001);

  // zero lon, non-zero lat accelerations
  Trajectory static_traj4(3, static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  static_traj4 << 0, 0, -1.75, -3.14 / 2.0, 0, 1, 1, -1.75, -3.14 / 2.0, 1, 2,
      1, +0, -3.14 / 2.0, 1;
  action = BehaviorStaticTrajectory::CalculateAction(2.0, observed_world,
                                                     static_traj4);
  lon_lat_action = boost::get<LonLatAction>(action);
  EXPECT_NEAR(lon_lat_action.acc_lon, 0.0, 0.001);
  EXPECT_NEAR(lon_lat_action.acc_lat, -1 / 2.0, 0.001);

  // non-zero lon, non-zero lat accelerations
  Trajectory static_traj5(3, static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  static_traj5 << 0, 0, -1.75, 3.14 / 4.0, 1, 1, 1, -1.75, 3.14 / 4.0, 1, 2, 1,
      +0, 3.14 / 4.0, 3;
  action = BehaviorStaticTrajectory::CalculateAction(2.0, observed_world,
                                                     static_traj5);
  lon_lat_action = boost::get<LonLatAction>(action);
  EXPECT_NEAR(lon_lat_action.acc_lon, sqrt(2.0) / 2.0, 0.001);
  EXPECT_NEAR(lon_lat_action.acc_lat, sqrt(2.0) / 2.0, 0.001);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}