// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include "gtest/gtest.h"
#include "modules/models/dynamic/single_track.hpp"
#include "modules/models/execution/interpolation/interpolate.hpp"
//#include "modules/models/execution/mpc/mpc.hpp"
#include "modules/commons/params/default_params.hpp"

using namespace modules::models::dynamic;
using namespace modules::models::execution;
using namespace modules::commons;

TEST(execution_model, execution_model_interpolate) {
  auto params = std::make_shared<DefaultParams>();

  Trajectory test_trajectory(11, (int)StateDefinition::MIN_STATE_SIZE);
  test_trajectory.col(StateDefinition::TIME_POSITION) = Eigen::ArrayXf::LinSpaced(11, 0, 10); // Time 0 to 10 seconds
  test_trajectory.col(StateDefinition::X_POSITION) = Eigen::ArrayXf::LinSpaced(11, 0, 10);
  ExecutionModelPtr exec_model(new ExecutionModelInterpolate(params));
  DynamicModelPtr dyn_model(new SingleTrackModel(params));

  State initial_state = test_trajectory.row(0);
  Trajectory followed_trajectory1 = exec_model->Execute(0.5, test_trajectory, dyn_model, initial_state);
  State next_state1 = followed_trajectory1.row(0);
  std::cout << "Interpolated state 1: " << next_state1 << std::endl;
  //  EXPECT_EQ(next_state1(StateDefinition::X_POSITION),0.5);

  Trajectory followed_trajectory2 = exec_model->Execute(4, test_trajectory, dyn_model, initial_state);
  State next_state2 = followed_trajectory2.row(0);
  std::cout << "Interpolated state 2: " << next_state2 << std::endl;
  //  EXPECT_EQ(next_state2(StateDefinition::X_POSITION),4);

  test_trajectory.col(StateDefinition::Y_POSITION) = Eigen::ArrayXf::LinSpaced(11, 0, 10);
  initial_state = test_trajectory.row(0);
  Trajectory followed_trajectory3 = exec_model->Execute(0.9, test_trajectory, dyn_model, initial_state);
  State next_state3 = followed_trajectory3.row(0);
  std::cout << "Interpolated state 3: " << next_state3 << std::endl;
  //   EXPECT_NEAR(next_state3(StateDefinition::X_POSITION),0.9,0.001);
  //   EXPECT_NEAR(next_state3(StateDefinition::Y_POSITION),0.9,0.001);
}

/*
TEST(execution_model, execution_model_mpc) {

  auto params = std::make_shared<DefaultParams>();

  Trajectory test_trajectory(3, (int)StateDefinition::MIN_STATE_SIZE);
  test_trajectory.col(StateDefinition::TIME_POSITION) = Eigen::ArrayXf::LinSpaced(3, 0, 10); // Time 0 to 10 seconds
  test_trajectory.col(StateDefinition::X_POSITION) = Eigen::ArrayXf::LinSpaced(3, 0, 10);
  test_trajectory.col(StateDefinition::Y_POSITION) = Eigen::ArrayXf::LinSpaced(3, 0, 0);
  test_trajectory.col(StateDefinition::THETA_POSITION) = Eigen::ArrayXf::LinSpaced(3, 0, 0);
  test_trajectory.col(StateDefinition::VEL_POSITION) = Eigen::ArrayXf::LinSpaced(3, 1, 1);

  ExecutionModelPtr exec_model(new ExecutionModelMpc(params));
  DynamicModelPtr dyn_model(new SingleTrackModel(params));

  Trajectory followed_trajectory1 = exec_model->Execute(0.5, test_trajectory, dyn_model, test_trajectory.row(0));
}
*/