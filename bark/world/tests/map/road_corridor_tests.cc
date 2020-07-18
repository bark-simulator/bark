// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/world/map/lane.hpp"
#include "bark/world/map/map_interface.hpp"
#include "bark/world/map/road.hpp"
#include "bark/world/map/road_corridor.hpp"
#include "bark/world/opendrive/opendrive.hpp"
#include "bark/world/tests/make_test_xodr_map.hpp"
#include "gtest/gtest.h"

TEST(road_corridor_tests, xodr_map_one_road_two_lanes) {
  using bark::world::map::MapInterface;
  using bark::world::map::MapInterfacePtr;
  using bark::world::map::RoadCorridorPtr;
  using bark::world::opendrive::OpenDriveMapPtr;
  using bark::world::opendrive::XodrDrivingDirection;
  using bark::world::opendrive::XodrRoadId;

  using bark::world::tests::MakeXodrMapOneRoadTwoLanes;

  OpenDriveMapPtr open_drive_map = MakeXodrMapOneRoadTwoLanes();

  MapInterfacePtr map_interface = std::make_shared<MapInterface>();
  map_interface->interface_from_opendrive(open_drive_map);

  std::vector<XodrRoadId> road_ids{100};
  XodrDrivingDirection driving_dir = XodrDrivingDirection::FORWARD;
  map_interface->GenerateRoadCorridor(road_ids, driving_dir);
  RoadCorridorPtr road_corridor =
      map_interface->GetRoadCorridor(road_ids, driving_dir);

  EXPECT_EQ(road_corridor->GetRoads().size(), 1);
}

TEST(road_corridor_tests, xodr_map_two_roads_one_lane) {
  using bark::world::map::MapInterface;
  using bark::world::map::MapInterfacePtr;
  using bark::world::map::RoadCorridorPtr;
  using bark::world::opendrive::OpenDriveMapPtr;
  using bark::world::opendrive::XodrDrivingDirection;
  using bark::world::opendrive::XodrRoadId;

  using bark::world::tests::MakeXodrMapTwoRoadsOneLane;

  OpenDriveMapPtr open_drive_map = MakeXodrMapTwoRoadsOneLane();

  MapInterfacePtr map_interface = std::make_shared<MapInterface>();
  map_interface->interface_from_opendrive(open_drive_map);

  std::vector<XodrRoadId> road_ids{100, 101};
  XodrDrivingDirection driving_dir = XodrDrivingDirection::FORWARD;
  map_interface->GenerateRoadCorridor(road_ids, driving_dir);
  RoadCorridorPtr road_corridor =
      map_interface->GetRoadCorridor(road_ids, driving_dir);

  EXPECT_EQ(road_corridor->GetRoads().size(), 2);
}

TEST(road_corridor_tests, xodr_map_ending_lane_in_parallel) {
  using bark::world::map::MapInterface;
  using bark::world::map::MapInterfacePtr;
  using bark::world::map::RoadCorridorPtr;
  using bark::world::opendrive::OpenDriveMapPtr;
  using bark::world::opendrive::XodrDrivingDirection;
  using bark::world::opendrive::XodrRoadId;

  using bark::world::tests::MakeXodrMapEndingLaneInParallel;

  OpenDriveMapPtr open_drive_map = MakeXodrMapEndingLaneInParallel();

  MapInterfacePtr map_interface = std::make_shared<MapInterface>();
  map_interface->interface_from_opendrive(open_drive_map);

  std::vector<XodrRoadId> road_ids{100};
  XodrDrivingDirection driving_dir = XodrDrivingDirection::FORWARD;
  map_interface->GenerateRoadCorridor(road_ids, driving_dir);
  RoadCorridorPtr road_corridor =
      map_interface->GetRoadCorridor(road_ids, driving_dir);

  EXPECT_EQ(road_corridor->GetRoads().size(), 1);
}
