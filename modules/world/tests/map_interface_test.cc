// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "gtest/gtest.h"
#include "modules/world/map/map_interface.hpp"
#include "modules/models/tests/make_test_world.hpp"

TEST(query_lanes, map_interface) {
  using namespace modules::world::opendrive;
  using namespace modules::world::map;
  using namespace modules::geometry;

  MapInterface map_interface =
    modules::models::tests::make_two_lane_map_interface();

  std::vector<XodrLanePtr> nearest_lanes;
  bool success = map_interface.FindNearestXodrLanes(
    Point2d(0, 0), 2, nearest_lanes);
  EXPECT_TRUE(success);
  EXPECT_EQ(nearest_lanes.size(), 2);

  success = map_interface.FindNearestXodrLanes(Point2d(0, 0), 3, nearest_lanes);
  EXPECT_TRUE(success);
  EXPECT_EQ(nearest_lanes.size(), 2);  // there exist only two lanes
}


TEST(point_in_lane, map_interface) {
  using namespace modules::world::opendrive;
  using namespace modules::world::map;
  using namespace modules::geometry;

  OpenDriveMapPtr open_drive_map(new OpenDriveMap());
  //! ROAD 1
  PlanViewPtr p(new PlanView());
  p->AddLine(Point2d(0.0f, 0.0f), 0.0f, 10.0f);

  //! XodrLane-Section 1
  XodrLaneSectionPtr ls(new XodrLaneSection(0.0));

  //! PlanView
  XodrLaneOffset off0 = {0.0f, 0.0f, 0.0f, 0.0f};
  XodrLaneWidth lane_width_0 = {0, 10, off0};
  XodrLanePtr lane0 =
    CreateLaneFromLaneWidth(0, p->GetReferenceLine(), lane_width_0, 0.05);
  lane0->SetLaneType(XodrLaneType::DRIVING);

  XodrLaneOffset off = {1.0f, 0.0f, 0.0f, 0.0f};
  XodrLaneWidth lane_width_1 = {0, 10, off};

  //! XodrLanes
  XodrLanePtr lane1 = CreateLaneFromLaneWidth(
    -1, p->GetReferenceLine(), lane_width_1, 0.05);
  lane1->SetLaneType(XodrLaneType::DRIVING);
  XodrLanePtr lane2 = CreateLaneFromLaneWidth(
    1, p->GetReferenceLine(), lane_width_1, 0.05);
  lane2->SetLaneType(XodrLaneType::DRIVING);
  XodrLanePtr lane3 = CreateLaneFromLaneWidth(
    2, lane2->GetLine(), lane_width_1, 0.05);
  lane3->SetLaneType(XodrLaneType::DRIVING);

  ls->AddLane(lane0);
  ls->AddLane(lane1);
  ls->AddLane(lane2);
  ls->AddLane(lane3);

  XodrRoadPtr r(new XodrRoad("highway", 100));
  r->SetPlanView(p);
  r->AddLaneSection(ls);

  open_drive_map->AddRoad(r);

  MapInterface map_interface;
  map_interface.SetOpenDriveMap(open_drive_map);

  std::vector<XodrLanePtr> nearest_lanes;
  Point2d point = Point2d(0.5, 0.5);
  bool success = map_interface.FindNearestXodrLanes(point, 3, nearest_lanes);

  success = map_interface.IsInXodrLane(point, (nearest_lanes.at(0))->GetId());
  EXPECT_FALSE(success);

  success = map_interface.IsInXodrLane(point, (nearest_lanes.at(1))->GetId());
  EXPECT_TRUE(success);

  success = map_interface.IsInXodrLane(point, (nearest_lanes.at(2))->GetId());
  EXPECT_FALSE(success);
}