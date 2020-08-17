// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/models/dynamic/single_track.hpp"
#include "bark/models/execution/interpolation/interpolate.hpp"
#include "gtest/gtest.h"
//#include "bark/models/execution/mpc/mpc.hpp"
#include "bark/commons/params/setter_params.hpp"

using namespace bark::models::dynamic;
using namespace bark::models::execution;
using namespace bark::commons;

TEST(execution_model, execution_model_interpolate) {
  auto params = std::make_shared<SetterParams>();

  Trajectory test_trajectory(11, (int)StateDefinition::MIN_STATE_SIZE);
  test_trajectory.col(StateDefinition::TIME_POSITION) =
      Eigen::ArrayXf::LinSpaced(11, 0, 10);  // Time 0 to 10 seconds
  test_trajectory.col(StateDefinition::X_POSITION) =
      Eigen::ArrayXf::LinSpaced(11, 0, 10);
  ExecutionModelPtr exec_model(new ExecutionModelInterpolate(params));
  DynamicModelPtr dyn_model(new SingleTrackModel(params));

  exec_model->Execute(0.5, test_trajectory, dyn_model);
  Trajectory followed_trajectory1 = exec_model->GetLastTrajectory();
  State next_state1 = followed_trajectory1.row(0);
  std::cout << "Interpolated state 1: " << next_state1 << std::endl;
  //  EXPECT_EQ(next_state1(StateDefinition::X_POSITION),0.5);

  exec_model->Execute(4, test_trajectory, dyn_model);
  Trajectory followed_trajectory2 = exec_model->GetLastTrajectory();
  State next_state2 = followed_trajectory2.row(0);
  std::cout << "Interpolated state 2: " << next_state2 << std::endl;
  //  EXPECT_EQ(next_state2(StateDefinition::X_POSITION),4);

  test_trajectory.col(StateDefinition::Y_POSITION) =
      Eigen::ArrayXf::LinSpaced(11, 0, 10);
  std::cout << test_trajectory << std::endl;
  exec_model->Execute(0.9, test_trajectory, dyn_model);
  Trajectory followed_trajectory3 = exec_model->GetLastTrajectory();
  State next_state3 = followed_trajectory3.row(0);
  std::cout << "Interpolated state 3: " << next_state3 << std::endl;
  //   EXPECT_NEAR(next_state3(StateDefinition::X_POSITION),0.9,0.001);
  //   EXPECT_NEAR(next_state3(StateDefinition::Y_POSITION),0.9,0.001);
}

/*
TEST(execution_model, execution_model_mpc) {

  auto params = std::make_shared<SetterParams>();

  Trajectory test_trajectory(3, (int)StateDefinition::MIN_STATE_SIZE);
  test_trajectory.col(StateDefinition::TIME_POSITION) =
Eigen::ArrayXf::LinSpaced(3, 0, 10); // Time 0 to 10 seconds
  test_trajectory.col(StateDefinition::X_POSITION) =
Eigen::ArrayXf::LinSpaced(3, 0, 10);
  test_trajectory.col(StateDefinition::Y_POSITION) =
Eigen::ArrayXf::LinSpaced(3, 0, 0);
  test_trajectory.col(StateDefinition::THETA_POSITION) =
Eigen::ArrayXf::LinSpaced(3, 0, 0);
  test_trajectory.col(StateDefinition::VEL_POSITION) =
Eigen::ArrayXf::LinSpaced(3, 1, 1);

  ExecutionModelPtr exec_model(new ExecutionModelMpc(params));
  DynamicModelPtr dyn_model(new SingleTrackModel(params));

  Trajectory followed_trajectory1 = exec_model->Execute(0.5, test_trajectory,
dyn_model, test_trajectory.row(0));
}
*/