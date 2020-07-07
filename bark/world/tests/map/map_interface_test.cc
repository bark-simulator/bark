// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/world/map/map_interface.hpp"
#include "bark/world/tests/make_test_xodr_map.hpp"
#include "gtest/gtest.h"

TEST(query_lanes, map_interface) {
  using bark::geometry::Point2d;
  using bark::world::map::MapInterface;
  using bark::world::opendrive::OpenDriveMapPtr;
  using bark::world::opendrive::XodrLanePtr;
  using bark::world::tests::MakeXodrMapOneRoadTwoLanes;

  OpenDriveMapPtr open_drive_map = MakeXodrMapOneRoadTwoLanes();

  bark::world::map::MapInterface map_interface;
  map_interface.interface_from_opendrive(open_drive_map);

  std::vector<XodrLanePtr> nearest_lanes;
  auto p = Point2d(-1.5, 0);
  bool success = map_interface.FindNearestXodrLanes(p, 2, nearest_lanes);
  EXPECT_TRUE(success);
  EXPECT_EQ(nearest_lanes.size(), 2);

  success = map_interface.FindNearestXodrLanes(p, 3, nearest_lanes);
  EXPECT_TRUE(success);
  EXPECT_EQ(nearest_lanes.size(), 2);  // there exist only two lanes
}

TEST(point_in_lane, map_interface) {
  using bark::geometry::Point2d;
  using bark::world::map::MapInterface;
  using bark::world::opendrive::OpenDriveMapPtr;
  using bark::world::opendrive::XodrLanePtr;
  using bark::world::tests::MakeXodrMapOneRoadTwoLanes;

  OpenDriveMapPtr open_drive_map = MakeXodrMapOneRoadTwoLanes();

  bark::world::map::MapInterface map_interface;
  map_interface.interface_from_opendrive(open_drive_map);

  std::vector<XodrLanePtr> nearest_lanes;
  Point2d point = Point2d(-0.5, 0.5);
  bool success = map_interface.FindNearestXodrLanes(point, 1, nearest_lanes);

  BARK_EXPECT_TRUE(success);
}