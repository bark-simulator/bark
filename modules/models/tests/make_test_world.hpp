// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_TESTS_MAKE_TEST_WORLD_HPP_
#define MODULES_MODELS_TESTS_MAKE_TEST_WORLD_HPP_

#include "modules/world/observed_world.hpp"
#include "modules/geometry/commons.hpp"
#include "modules/world/goal_definition/goal_definition.hpp"
#include "modules/world/goal_definition/goal_definition_polygon.hpp"
#include "modules/world/map/map_interface.hpp"

namespace modules {
namespace models {
namespace tests {

class DummyMapInterface : public modules::world::map::MapInterface {
  virtual std::pair<modules::geometry::Point2d, modules::geometry::Point2d> BoundingBox() const {
       return std::make_pair(modules::geometry::Point2d(-100,-100), modules::geometry::Point2d(3000,3000));}
};


modules::world::WorldPtr make_test_world(int num_other_agents, double rel_distance, double ego_velocity, double velocity_difference,
                                         const modules::world::goal_definition::GoalDefinitionPtr& ego_goal_definition =
                                         std::make_shared<modules::world::goal_definition::GoalDefinitionPolygon>());

modules::world::ObservedWorld make_test_observed_world(int num_other_agents,
                                   double rel_distance, double ego_velocity,
                                   double velocity_difference,
                                   const modules::world::goal_definition::GoalDefinitionPtr& ego_goal_definition =
                                         std::make_shared<modules::world::goal_definition::GoalDefinitionPolygon>());

modules::world::map::MapInterface make_two_lane_map_interface();

} // namespace tests
} // namespace models
} // namespace modules

#endif  // MODULES_MODELS_TESTS_MAKE_TEST_WORLD_HPP_