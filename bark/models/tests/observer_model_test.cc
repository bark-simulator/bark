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
#include "bark/world/tests/make_test_world.hpp"
#include "bark/commons/params/setter_params.hpp"

using bark::commons::SetterParams;
using bark::geometry::Polygon;
using bark::geometry::Point2d;
using bark::geometry::standard_shapes::GenerateGoalRectangle;
using bark::world::goal_definition::GoalDefinitionPolygon;
using bark::world::objects::AgentId;
using bark::world::tests::make_test_world;
using namespace bark::world;

TEST(observer_model_none, base_test) {
  using bark::models::observer::ObserverModelNone;
  auto params = std::make_shared<SetterParams>();
  ObserverModelNone observer_none(params);

  Polygon polygon = GenerateGoalRectangle(6,3);
  std::shared_ptr<Polygon> goal_polygon(
    std::dynamic_pointer_cast<Polygon>(polygon.Translate(
      Point2d(50, -2))));  // < move the goal polygon into the driving
                            // corridor in front of the ego vehicle
  auto goal_definition_ptr =
      std::make_shared<GoalDefinitionPolygon>(*goal_polygon);
  double ego_velocity = 5.0, rel_distance = 2.0, velocity_difference = 2.0;
  WorldPtr world = make_test_world(0, rel_distance, ego_velocity,
                                   velocity_difference, goal_definition_ptr);

  ObservedWorld observed_world = observer_none.Observe(world, AgentId(1));

  // NOTE: assert ObservedWorld is generated for the correct AgentId
  EXPECT_EQ(observed_world.GetEgoAgentId(), AgentId(1));
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
