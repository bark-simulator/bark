// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "gtest/gtest.h"
#include "modules/world/map/lane.hpp"
#include "modules/world/map/map_interface.hpp"
#include "modules/world/map/road.hpp"
#include "modules/world/map/road_corridor.hpp"
#include "modules/world/opendrive/opendrive.hpp"
#include "modules/world/tests/make_test_xodr_map.hpp"

TEST(road_corridor_tests, xodr_map_one_road_two_lanes) {
  using modules::world::map::MapInterface;
  using modules::world::map::MapInterfacePtr;
  using modules::world::map::RoadCorridorPtr;
  using modules::world::opendrive::OpenDriveMapPtr;
  using modules::world::opendrive::XodrDrivingDirection;
  using modules::world::opendrive::XodrRoadId;

  using modules::world::tests::MakeXodrMapOneRoadTwoLanes;

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

TEST(road_corridor_tests, xodr_map_two_roads_one_lanes) {
  using modules::world::map::MapInterface;
  using modules::world::map::MapInterfacePtr;
  using modules::world::map::RoadCorridorPtr;
  using modules::world::opendrive::OpenDriveMapPtr;
  using modules::world::opendrive::XodrDrivingDirection;
  using modules::world::opendrive::XodrRoadId;

  using modules::world::tests::MakeXodrMapTwoRoadsOneLane;

  OpenDriveMapPtr open_drive_map = MakeXodrMapTwoRoadsOneLane();

  MapInterfacePtr map_interface = std::make_shared<MapInterface>();
  map_interface->interface_from_opendrive(open_drive_map);

  std::vector<XodrRoadId> road_ids{100};
  XodrDrivingDirection driving_dir = XodrDrivingDirection::FORWARD;
  map_interface->GenerateRoadCorridor(road_ids, driving_dir);
  RoadCorridorPtr road_corridor =
      map_interface->GetRoadCorridor(road_ids, driving_dir);

  EXPECT_EQ(road_corridor->GetRoads().size(), 1);
}
