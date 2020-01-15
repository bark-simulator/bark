// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "gtest/gtest.h"
#include "modules/world/map/lane.hpp"
#include "modules/world/map/road.hpp"
#include "modules/world/map/road_corridor.hpp"
#include "modules/geometry/geometry.hpp"
#include "modules/world/opendrive/opendrive.hpp"
#include "modules/world/map/map_interface.hpp"
#include "modules/models/tests/make_test_world.hpp"


TEST(lane_road_tests, lane) {
  using modules::world::opendrive::XodrLanePtr;
  using modules::world::opendrive::OpenDriveMapPtr;
  using modules::world::map::MapInterface;
  using modules::world::map::Lane;
  using modules::world::map::LanePtr;
  using modules::world::map::Lanes;
  using modules::world::map::Road;
  using modules::world::map::Roads;
  using modules::world::map::RoadPtr;

  using modules::geometry::Point2d;
  using modules::models::tests::make_map_interface_two_connected_roads;

  MapInterface map_interface = make_map_interface_two_connected_roads();
  OpenDriveMapPtr open_drive_map = map_interface.get_open_drive_map();

  // convert xodr to lane and road
  Roads roads;
  for (auto xodr_road : open_drive_map->get_roads()) {
    std::cout << "RoadId: "<< xodr_road.second->get_id() << std::endl;
    RoadPtr road = std::make_shared<Road>(xodr_road.second);
    Lanes lanes;
    for (auto xodr_lane : xodr_road.second->get_lanes()) {
      std::cout << "LaneId: "<< xodr_lane.second->get_id() << std::endl;
      LanePtr lane = std::make_shared<Lane>(xodr_lane.second);
      lanes[xodr_lane.second->get_id()] = lane;
    }
    road->SetLanes(lanes);
    roads[xodr_road.second->get_id()] = road;
  }

}



TEST(lane_corridor_tests, lane_corridors) {
  using modules::world::opendrive::XodrLanePtr;
  using modules::world::opendrive::OpenDriveMapPtr;
  using modules::world::map::MapInterface;
  using modules::geometry::Point2d;
  using modules::models::tests::make_two_lane_map_interface;

  MapInterface map_interface = make_two_lane_map_interface();
  OpenDriveMapPtr open_drive_map = map_interface.get_open_drive_map();

  // TODO(@hart): create LaneCorridor
}


TEST(road_corridors_tests, road_corridors) {
  using modules::world::opendrive::XodrLanePtr;
  using modules::world::opendrive::OpenDriveMapPtr;
  using modules::world::map::MapInterface;
  using modules::geometry::Point2d;
  using modules::models::tests::make_two_lane_map_interface;

  MapInterface map_interface = make_two_lane_map_interface();
  OpenDriveMapPtr open_drive_map = map_interface.get_open_drive_map();

  // TODO(@hart): create RoadCorridor
}