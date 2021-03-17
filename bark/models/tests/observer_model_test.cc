// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#include "gtest/gtest.h"
#include "bark/models/observer/observer_model.hpp"
#include "bark/models/observer/observer_model_none.hpp"
// #include "bark/world/tests/make_test_world.hpp"
#include "bark/commons/params/setter_params.hpp"

using bark::commons::SetterParams;

TEST(observer_model, IF_tests) {
  auto params = std::make_shared<SetterParams>();

}

TEST(observer_model_none, base_test) {
  using bark::models::observer::ObserverModelNone;
  auto params = std::make_shared<SetterParams>();
  ObserverModelNone observer_none(params);

  // Polygon polygon = GenerateGoalRectangle(6,3);
  // std::shared_ptr<Polygon> goal_polygon(
  //     std::dynamic_pointer_cast<Polygon>(polygon.Translate(
  //         Point2d(50, -2))));  // < move the goal polygon into the driving
  //                              // corridor in front of the ego vehicle
  // auto goal_definition_ptr =
  //     std::make_shared<GoalDefinitionPolygon>(*goal_polygon);
  // double ego_velocity = 5.0, rel_distance = 2.0, velocity_difference = 2.0;
  // // TODO: pass goal polygon to make_world_test
  // WorldPtr world = make_test_world(0, rel_distance, ego_velocity,
  //                                  velocity_difference, goal_definition_ptr);


  // observer_none.Observer();
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
