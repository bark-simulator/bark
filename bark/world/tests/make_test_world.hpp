// Copyright (c) 2020 Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_MODELS_TESTS_MAKE_TEST_WORLD_HPP_
#define BARK_MODELS_TESTS_MAKE_TEST_WORLD_HPP_

#include "bark/geometry/commons.hpp"
#include "bark/world/goal_definition/goal_definition.hpp"
#include "bark/world/goal_definition/goal_definition_polygon.hpp"
#include "bark/world/map/map_interface.hpp"
#include "bark/world/observed_world.hpp"

namespace bark {
namespace world {
namespace tests {

using bark::geometry::Line;
using bark::geometry::Point2d;
using bark::geometry::Polygon;
using bark::geometry::Pose;
using bark::world::ObservedWorld;
using bark::world::WorldPtr;
using bark::world::goal_definition::GoalDefinitionPolygon;
using bark::world::goal_definition::GoalDefinitionPtr;
using bark::world::map::LaneCorridor;
using bark::world::map::LaneCorridorPtr;

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
}  // namespace bark

#endif  // BARK_MODELS_TESTS_MAKE_TEST_WORLD_HPP_