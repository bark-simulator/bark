// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_TESTS_MAKE_TEST_WORLD_HPP_
#define MODULES_MODELS_TESTS_MAKE_TEST_WORLD_HPP_

#include "modules/world/observed_world.hpp"
#include "modules/geometry/commons.hpp"
#include "modules/world/goal_definition/goal_definition.hpp"

namespace modules {
namespace models {
namespace tests {

class DummyMapInterface : public modules::world::map::MapInterface {
  virtual std::pair<modules::geometry::Point2d, modules::geometry::Point2d> BoundingBox() const {
       return std::make_pair(modules::geometry::Point2d(-100,-100), modules::geometry::Point2d(100,3000));}
};


modules::world::WorldPtr make_test_world(int num_other_agents, double rel_distance, double ego_velocity, double velocity_difference,
                                         const modules::world::goal_definition::GoalDefinition& ego_goal_definition = modules::world::goal_definition::GoalDefinition());

modules::world::ObservedWorld make_test_observed_world(int num_other_agents,
                                   double rel_distance, double ego_velocity,
                                   double velocity_difference,
                                   const modules::world::goal_definition::GoalDefinition& ego_goal_definition = modules::world::goal_definition::GoalDefinition());

} // namespace tests
} // namespace models
} // namespace modules

#endif  // MODULES_MODELS_TESTS_MAKE_TEST_WORLD_HPP_