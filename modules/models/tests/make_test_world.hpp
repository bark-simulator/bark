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

using modules::geometry::Polygon;
using modules::geometry::Line;
using modules::geometry::Pose;
using modules::geometry::Point2d;
using modules::world::map::LaneCorridor;
using modules::world::map::LaneCorridorPtr;

class DummyMapInterface : public modules::world::map::MapInterface {
  virtual std::pair<modules::geometry::Point2d, modules::geometry::Point2d> BoundingBox() const {
       return std::make_pair(modules::geometry::Point2d(-100,-100), modules::geometry::Point2d(3000,3000));}
};

class DummyRoadCorridor : public modules::world::map::RoadCorridor {
public:
  DummyRoadCorridor(const Line& driving_corridor_center,
                    const Line& driving_corridor_left_boundary,
                    const Line& driving_corridor_right_boundary) :
                    corridor_ptr_() {
    Line polygon_line = driving_corridor_left_boundary;
    Line temp = driving_corridor_right_boundary;
    temp.reverse();
    polygon_line.append_linestring(temp);
    const auto corridor_polygon = Polygon(Pose(), polygon_line);

    LaneCorridor lane_corridor;
    lane_corridor.SetLeftBoundary(driving_corridor_left_boundary);
    lane_corridor.SetRightBoundary(driving_corridor_right_boundary);
    lane_corridor.SetCenterLine(driving_corridor_center);
    lane_corridor.SetMergedPolygon(corridor_polygon);
    corridor_ptr_ = std::make_shared<LaneCorridor>(lane_corridor);
  }
  virtual ~DummyRoadCorridor() {}

  virtual LaneCorridorPtr GetCurrentLaneCorridor(const Point2d& pt) const {
    return corridor_ptr_;
  }

  private:
    LaneCorridorPtr corridor_ptr_;
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

modules::world::map::MapInterface make_map_interface_two_connected_roads();

} // namespace tests
} // namespace models
} // namespace modules

#endif  // MODULES_MODELS_TESTS_MAKE_TEST_WORLD_HPP_