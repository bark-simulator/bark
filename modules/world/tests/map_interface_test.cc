// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "gtest/gtest.h"
#include "modules/world/map/map_interface.hpp"

void build_two_road_junction_map(const modules::world::opendrive::OpenDriveMapPtr &map) {
  using namespace std;
  using namespace modules::world::opendrive;
  using namespace modules::geometry;

  //! ROAD 1
  PlanViewPtr p(new PlanView());
  p->add_line(Point2d(0.0f, 0.0f), 0.0f, 10.0f);

  //! Road-Link
  RoadLinkInfo pre;
  pre.id_ = 1;
  pre.type_ = "road";

  RoadLink l;  // can either link to another road or to a junction
  l.set_predecessor(pre);
  l.set_successor(pre);

  //! Lane-Section 1
  LaneSectionPtr ls(new LaneSection(0.0));

  //! Lane-Section 2
  LaneSectionPtr ls2(new LaneSection(10.0));

  //! Lane
  LaneOffset off = {1.0f, 0.0f, 0.0f, 0.0f};
  LaneWidth lane_width_1 = {0, 10, off};
  LanePtr lane = create_lane_from_lane_width(-1, p->get_reference_line(), lane_width_1, 0.05);
  lane->set_lane_type(LaneType::DRIVING);
  LanePtr lane2 = create_lane_from_lane_width(1, p->get_reference_line(), lane_width_1, 0.05);
  lane2->set_lane_type(LaneType::DRIVING);

  ls->add_lane(lane);
  ls->add_lane(lane2);
  ls2->add_lane(lane);
  ls2->add_lane(lane2);

  RoadPtr r(new Road("highway", 100));
  r->set_plan_view(p);
  r->set_link(l);
  r->add_lane_section(ls);

  //! ROAD 2
  PlanViewPtr p2(new PlanView());
  p2->add_line(Point2d(0.0f, 0.0f), 0.0f, 10.0f);

  //! Road-Link
  RoadLink l2 = {};  // can either link to another road or to a junction

  //! Lane-Section 1
  LaneSectionPtr ls3(new LaneSection(0.0));

  //! Lane
  LaneOffset off2 = {1.0f, 0.0f, 0.0f, 0.0f};
  LaneWidth lane_width_2 = {0, 10, off2};

  LanePtr lane3 = create_lane_from_lane_width(-1, p2->get_reference_line(), lane_width_2, 0.05);
  lane3->set_lane_type(LaneType::DRIVING);
  LanePtr lane4 = create_lane_from_lane_width(1, p2->get_reference_line(), lane_width_2, 0.05);
  lane4->set_lane_type(LaneType::DRIVING);

  ls3->add_lane(lane3);
  ls3->add_lane(lane4);

  //! new road
  RoadPtr r2(new Road("highway", 200));
  r2->set_plan_view(p2);
  r2->set_link(l2);
  r2->add_lane_section(ls3);

  // connect roads
  JunctionPtr j(new Junction("kreuz", 1));
  Connection con;
  con.incoming_road_ = 100;
  con.connecting_road_ = 200;
  con.id_ = 1;

  LaneLink link_1 = {-1, -1};
  LaneLink link_2 = {1, 1};

  con.add_lane_link(link_1);
  con.add_lane_link(link_2);
  j->add_connection(con);

  map->add_road(r);
  map->add_road(r2);
  map->add_junction(j);
}



TEST(query_lanes, map_interface) {
  using namespace modules::world::opendrive;
  using namespace modules::world::map;
  using namespace modules::geometry;

  OpenDriveMapPtr map(new OpenDriveMap());
  build_two_road_junction_map(map);

  MapInterface map_interface;
  map_interface.interface_from_opendrive(map);
  std::vector<LanePtr> nearest_lanes;
  bool success = map_interface.FindNearestLanes(Point2d(0, 0), 2, nearest_lanes);
  EXPECT_TRUE(success);
  EXPECT_EQ(nearest_lanes.size(), (uint) 2);

  success = map_interface.FindNearestLanes(Point2d(0, 0), 3, nearest_lanes);
  EXPECT_TRUE(success);
  EXPECT_EQ(nearest_lanes.size(), (uint) 3);
}


TEST(point_in_lane, map_interface) {
  using namespace modules::world::opendrive;
  using namespace modules::world::map;
  using namespace modules::geometry;

  OpenDriveMapPtr open_drive_map(new OpenDriveMap());
    //! ROAD 1
  PlanViewPtr p(new PlanView());
  p->add_line(Point2d(0.0f, 0.0f), 0.0f, 10.0f);

  //! Lane-Section 1
  LaneSectionPtr ls(new LaneSection(0.0));

  //! PlanView
  LaneOffset off0 = {0.0f, 0.0f, 0.0f, 0.0f};
  LaneWidth lane_width_0 = {0, 10, off0};
  LanePtr lane0 = create_lane_from_lane_width(0, p->get_reference_line(), lane_width_0, 0.05);
  lane0->set_lane_type(LaneType::DRIVING);

  LaneOffset off = {1.0f, 0.0f, 0.0f, 0.0f};
  LaneWidth lane_width_1 = {0, 10, off};

  //! Lanes
  LanePtr lane1 = create_lane_from_lane_width(-1, p->get_reference_line(), lane_width_1, 0.05);
  lane1->set_lane_type(LaneType::DRIVING);
  LanePtr lane2 = create_lane_from_lane_width(1, p->get_reference_line(), lane_width_1, 0.05);
  lane2->set_lane_type(LaneType::DRIVING);
  LanePtr lane3 = create_lane_from_lane_width(2, lane2->get_line(), lane_width_1, 0.05);
  lane3->set_lane_type(LaneType::DRIVING);

  ls->add_lane(lane0);
  ls->add_lane(lane1);
  ls->add_lane(lane2);
  ls->add_lane(lane3);

  RoadPtr r(new Road("highway", 100));
  r->set_plan_view(p);
  r->add_lane_section(ls);

  open_drive_map->add_road(r);
  
  RoadgraphPtr roadgraph(new Roadgraph());
  roadgraph->Generate(open_drive_map);

  MapInterface map_interface;
  map_interface.set_open_drive_map(open_drive_map);
  map_interface.set_roadgraph(roadgraph);

  std::vector<LanePtr> nearest_lanes;
  Point2d point = Point2d(0.5, 0.5);
  bool success = map_interface.FindNearestLanes(point, 3, nearest_lanes);

  success = map_interface.isInLane(point, (nearest_lanes.at(0))->get_id());
  EXPECT_FALSE(success);

  success = map_interface.isInLane(point, (nearest_lanes.at(1))->get_id());
  EXPECT_TRUE(success);

  success = map_interface.isInLane(point, (nearest_lanes.at(2))->get_id());
  EXPECT_FALSE(success);

  bool success_corr = map_interface.ComputeAllDrivingCorridors();

}