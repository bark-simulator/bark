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


TEST(road_corridor_tests, basic_road_corridor) {
  using modules::world::opendrive::XodrLanePtr;
  using modules::world::opendrive::OpenDriveMapPtr;
  using modules::world::map::MapInterface;
  using modules::world::map::Lane;
  using modules::world::map::LaneId;
  using modules::world::map::LanePtr;
  using modules::world::map::Lanes;
  using modules::world::map::Road;
  using modules::world::map::RoadId;
  using modules::world::map::Roads;
  using modules::world::map::RoadPtr;
  using modules::world::map::RoadCorridor;
  using modules::world::map::RoadCorridorPtr;

  using modules::geometry::Point2d;
  using modules::geometry::Line;
  using modules::models::tests::make_map_interface_two_connected_roads;

  MapInterface map_interface = make_map_interface_two_connected_roads();
  OpenDriveMapPtr open_drive_map = map_interface.get_open_drive_map();

  // convert xodr to lane and road
  Roads roads;
  for (auto xodr_road : open_drive_map->get_roads()) {
    // std::cout << "RoadId: "<< xodr_road.second->get_id() << std::endl;
    RoadPtr road = std::make_shared<Road>(xodr_road.second);
    Lanes lanes;
    for (auto xodr_lane : xodr_road.second->get_lanes()) {
      // std::cout << "LaneId: "<< xodr_lane.second->get_id() << std::endl;
      LanePtr lane = std::make_shared<Lane>(xodr_lane.second);
      lanes[xodr_lane.second->get_id()] = lane;
    }
    road->SetLanes(lanes);
    roads[xodr_road.first] = road;
  }

  // road connections
  roads[100]->SetNextRoad(roads[101]);
  roads[101]->SetNextRoad(nullptr);

  // set lane links
  roads[100]->GetLane(1)->SetNextLane(roads[101]->GetLane(4));
  roads[100]->GetLane(2)->SetNextLane(roads[101]->GetLane(5));
  roads[100]->GetLane(3)->SetNextLane(roads[101]->GetLane(6));

  // basic asserts
  EXPECT_EQ(roads.size(), 2);
  EXPECT_EQ(roads[100]->GetLanes().size(), 3);
  EXPECT_EQ(roads[100]->GetLane(1)->get_id(), 1);
  EXPECT_EQ(roads[100]->GetLane(2)->get_id(), 2);
  EXPECT_EQ(roads[100]->GetLane(3)->get_id(), 3);
  EXPECT_EQ(roads[101]->GetLanes().size(), 3);
  EXPECT_EQ(roads[101]->GetLane(4)->get_id(), 4);
  EXPECT_EQ(roads[101]->GetLane(5)->get_id(), 5);
  EXPECT_EQ(roads[101]->GetLane(6)->get_id(), 6);

  // link asserts
  EXPECT_EQ(roads[100]->GetNextRoad(), roads[101]);
  EXPECT_EQ(roads[101]->GetNextRoad(), nullptr);
  EXPECT_EQ(
    roads[100]->GetLane(1)->GetNextLane(),
    roads[101]->GetLane(4));
  EXPECT_EQ(
    roads[100]->GetLane(2)->GetNextLane(),
    roads[101]->GetLane(5));
  EXPECT_EQ(
    roads[100]->GetLane(3)->GetNextLane(),
    roads[101]->GetLane(6));

  RoadCorridorPtr road_corridor = std::make_shared<RoadCorridor>();
  road_corridor->SetRoads(roads);

  // compute stuff
}


TEST(road_corridor_tests, lane_corridor_calculation) {
  using modules::world::opendrive::XodrLanePtr;
  using modules::world::opendrive::OpenDriveMapPtr;
  using modules::world::map::MapInterface;
  using modules::world::map::Lane;
  using modules::world::map::LaneId;
  using modules::world::map::LanePtr;
  using modules::world::map::Lanes;
  using modules::world::map::Road;
  using modules::world::map::RoadId;
  using modules::world::map::Roads;
  using modules::world::map::RoadPtr;
  using modules::world::map::RoadCorridor;
  using modules::world::map::RoadCorridorPtr;

  using modules::geometry::Point2d;
  using modules::geometry::Line;
  using modules::models::tests::make_map_interface_two_connected_roads;

  MapInterface map_interface = make_map_interface_two_connected_roads();
  OpenDriveMapPtr open_drive_map = map_interface.get_open_drive_map();

  // convert xodr to lane and road
  Roads roads;
  for (auto xodr_road : open_drive_map->get_roads()) {
    // std::cout << "RoadId: "<< xodr_road.second->get_id() << std::endl;
    RoadPtr road = std::make_shared<Road>(xodr_road.second);
    Lanes lanes;
    for (auto xodr_lane : xodr_road.second->get_lanes()) {
      // std::cout << "LaneId: "<< xodr_lane.second->get_id() << std::endl;
      LanePtr lane = std::make_shared<Lane>(xodr_lane.second);
      lanes[xodr_lane.second->get_id()] = lane;
    }
    road->SetLanes(lanes);
    roads[xodr_road.first] = road;
  }

  // road connections
  roads[100]->SetNextRoad(roads[101]);
  roads[101]->SetNextRoad(nullptr);

  // set successors
  roads[100]->GetLane(1)->SetNextLane(roads[101]->GetLane(4));
  roads[100]->GetLane(2)->SetNextLane(roads[101]->GetLane(5));
  roads[100]->GetLane(3)->SetNextLane(roads[101]->GetLane(6));

  // TODO(@hart): set left and right neighbour

  // TODO(@hart): set centerline

  // TODO(@hart): set boundaries

  // TODO(@hart): set polygons


  // basic asserts
  EXPECT_EQ(roads.size(), 2);
  EXPECT_EQ(roads[100]->GetLanes().size(), 3);
  EXPECT_EQ(roads[100]->GetLane(6)->get_id(), 6);
  EXPECT_EQ(roads[100]->GetLane(7)->get_id(), 7);
  EXPECT_EQ(roads[100]->GetLane(8)->get_id(), 8);
  EXPECT_EQ(roads[101]->GetLanes().size(), 3);
  EXPECT_EQ(roads[101]->GetLane(9)->get_id(), 9);
  EXPECT_EQ(roads[101]->GetLane(10)->get_id(), 10);
  EXPECT_EQ(roads[101]->GetLane(11)->get_id(), 11);

  // link asserts
  EXPECT_EQ(roads[100]->GetNextRoad(), roads[101]);
  EXPECT_EQ(roads[101]->GetNextRoad(), nullptr);
  EXPECT_EQ(
    roads[100]->GetLane(6)->GetNextLane(),
    roads[101]->GetLane(9));
  EXPECT_EQ(
    roads[100]->GetLane(7)->GetNextLane(),
    roads[101]->GetLane(10));
  EXPECT_EQ(
    roads[100]->GetLane(8)->GetNextLane(),
    roads[101]->GetLane(11));

  RoadCorridorPtr road_corridor = std::make_shared<RoadCorridor>();
  road_corridor->SetRoads(roads);

  // TODO(@hart): calculate LaneCorridors
  // TODO(@hart): assert LaneCorridors
}