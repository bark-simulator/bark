// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_TESTS_MAKE_TEST_WORLD_HPP_
#define MODULES_MODELS_TESTS_MAKE_TEST_WORLD_HPP_

#include "modules/geometry/commons.hpp"
#include "modules/world/goal_definition/goal_definition.hpp"
#include "modules/world/goal_definition/goal_definition_polygon.hpp"
#include "modules/world/map/map_interface.hpp"
#include "modules/world/observed_world.hpp"

namespace modules {
namespace world {
namespace tests {

using modules::geometry::Line;
using modules::geometry::Point2d;
using modules::geometry::Polygon;
using modules::geometry::Pose;
using modules::world::ObservedWorld;
using modules::world::WorldPtr;
using modules::world::goal_definition::GoalDefinitionPolygon;
using modules::world::goal_definition::GoalDefinitionPtr;
using modules::world::map::LaneCorridor;
using modules::world::map::LaneCorridorPtr;

WorldPtr make_test_world(int num_other_agents, double rel_distance,
                         double ego_velocity, double velocity_difference,
                         const GoalDefinitionPtr& ego_goal_definition =
                             std::make_shared<GoalDefinitionPolygon>());

ObservedWorld make_test_observed_world(
    int num_other_agents, double rel_distance, double ego_velocity,
    double velocity_difference,
    const GoalDefinitionPtr& ego_goal_definition =
        std::make_shared<GoalDefinitionPolygon>());

WorldPtr MakeTestWorldHighway();

}  // namespace tests
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_TESTS_MAKE_TEST_WORLD_HPP_