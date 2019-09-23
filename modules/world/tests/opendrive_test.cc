// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include "gtest/gtest.h"

#include "modules/geometry/commons.hpp"
#include "modules/geometry/line.hpp"
#include "modules/world/opendrive/plan_view.hpp"
#include "modules/world/opendrive/road.hpp"
#include "modules/world/opendrive/lane_section.hpp"
#include "modules/world/opendrive/junction.hpp"
#include "modules/world/opendrive/opendrive.hpp"
#include "modules/world/opendrive/commons.hpp"

TEST(create_plan_view, open_drive) {
  using namespace std;
  using namespace modules::world::opendrive;
  using namespace modules::geometry;

  //! new plan view
  PlanView p;

  //! add line
  p.add_line(Point2d(0.0f, 0.0f), 1.5, 10.0f);

  //! add arc
  p.add_arc(Point2d(0.0f, 0.0f), 0.0f, 10.0f, 0.1f);

  //! add spiral
  p.add_spiral(Point2d(0.0f, 0.0f), 0.0f, 1.0f, 0.0f, 0.7f);

  //p.print_points();
}

TEST(lane, open_drive) {
  using namespace std;
  using namespace modules::world::opendrive;
  using namespace modules::geometry;

  //! new plan view
  PlanView p;
  LaneOffset off = {1.5f, 0.0f, 0.0f, 0.0f};

  //! vertical
  p.add_line(Point2d(0.0f, 0.0f), 1.5707, 10.0f);
  LaneWidth lane_width = {0, 10.0, off};

  LanePtr lane = create_lane_from_lane_width(1, p.get_reference_line(), lane_width, 0.05f); // left side

  lane->set_lane_type(LaneType::DRIVING);
  EXPECT_EQ(lane->get_lane_type(), LaneType::DRIVING);

  Line line = lane->get_line();

  EXPECT_NEAR(bg::get<0>(line.obj_[0]), -1.5, 0.1);
  EXPECT_NEAR(bg::get<1>(line.obj_[0]), 0.0, 0.1);
  EXPECT_NEAR(bg::get<0>(line.obj_[line.obj_.size() - 1]), -1.5, 0.1);
  EXPECT_NEAR(bg::get<1>(line.obj_[line.obj_.size() - 1]), 10.0, 0.1);

  lane = create_lane_from_lane_width(-1, p.get_reference_line(), lane_width, 0.05f); // right side

  line = lane->get_line();
  
  std::cout << bg::get<1>(line.obj_[line.obj_.size() - 1]) << std::endl;

  EXPECT_NEAR(bg::get<0>(line.obj_[0]), 1.5, 0.1);
  EXPECT_NEAR(bg::get<1>(line.obj_[0]), 0.0, 0.1);
  EXPECT_NEAR(bg::get<0>(line.obj_[line.obj_.size() - 1]), 1.5, 0.1);
  EXPECT_NEAR(bg::get<1>(line.obj_[line.obj_.size() - 1]), 10.0, 0.1);

  //! new plan view
  PlanView p2;

  //! horizontal
  p2.add_line(Point2d(0.0f, 0.0f), 0.0f, 10.0f);

  lane = create_lane_from_lane_width(1, p2.get_reference_line(), lane_width, 0.05f); // left side

  line = lane->get_line();

  EXPECT_NEAR(bg::get<0>(line.obj_[0]), 0.0, 0.1);
  EXPECT_NEAR(bg::get<1>(line.obj_[0]), 1.5, 0.1);
  EXPECT_NEAR(bg::get<0>(line.obj_[line.obj_.size() - 1]), 10.0, 0.1);
  EXPECT_NEAR(bg::get<1>(line.obj_[line.obj_.size() - 1]), 1.5, 0.1);

  lane = create_lane_from_lane_width(-1, p2.get_reference_line(), lane_width, 0.05f); // right side

  line = lane->get_line();
  EXPECT_NEAR(bg::get<0>(line.obj_[0]), 0.0, 0.1);
  EXPECT_NEAR(bg::get<1>(line.obj_[0]), -1.5, 0.1);
  EXPECT_NEAR(bg::get<0>(line.obj_[line.obj_.size() - 1]), 10.0, 0.1);
  EXPECT_NEAR(bg::get<1>(line.obj_[line.obj_.size() - 1]), -1.5, 0.1);

  // spiral and arc tests are ommitted due to their complexits -> verify with plots!
}

TEST(road, open_drive) {
  using namespace std;
  using namespace modules::world::opendrive;
  using namespace modules::geometry;

  //! new plan view
  PlanViewPtr p(new PlanView());
  p->add_line(Point2d(0.0f, 0.0f), 1.5707, 10.0f);

  RoadLinkInfo pre;
  pre.id_ = 2;
  pre.type_ = "road";

  RoadLinkInfo suc;
  suc.id_ = 1;
  suc.type_ = "road";

  //! Road-Link
  RoadLink l;  // can either link to another road or to a junction
  l.set_predecessor(pre);
  l.set_successor(suc);

  //! Lane-Section 1
  LaneSectionPtr ls(new LaneSection(0.0));

  //! Lane-Section 2
  LaneSectionPtr ls2(new LaneSection(10.0));

  //! Lane
  LaneOffset off = {1.0f, 0.0f, 0.0f, 0.0f};
  LaneWidth lane_width = {0, 1, off};
  LanePtr lane = create_lane_from_lane_width(-1, p->get_reference_line(), lane_width, 0.05f);
  LanePtr lane2 = create_lane_from_lane_width(1, p->get_reference_line(), lane_width, 0.05f);

  ls->add_lane(lane);
  ls2->add_lane(lane2);

  //! new road
  Road r("highway", 1);
  r.set_plan_view(p);
  r.set_link(l);
  r.add_lane_section(ls);
}

TEST(junction, open_drive) {
  using namespace std;
  using namespace modules::world::opendrive;
  using namespace modules::geometry;

  Junction j("kreuz", 1);

  Connection con;
  con.incoming_road_ = 100;
  con.connecting_road_ = 200;
  con.id_ = 1;

  LaneLink l1 = {1, -1};
  LaneLink l2 = {2, 2};

  con.add_lane_link(l1);
  con.add_lane_link(l2);

  j.add_connection(con);
}

TEST(map, open_drive) {
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
  LaneWidth lane_width_1 = {0, 10.0, off};

  LanePtr lane = create_lane_from_lane_width(-1, p->get_reference_line(), lane_width_1, 0.05f);
  LanePtr lane2 = create_lane_from_lane_width(1, p->get_reference_line(), lane_width_1, 0.05f);

  RoadMark rm {roadmark::RoadMarkType::SOLID, roadmark::RoadMarkColor::STANDARD, 0.1};

  std::cout << print(rm) << std::endl;
  lane2->set_road_mark(rm);

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
  LaneWidth lane_width_2 = {0, 10.0, off2};

  LanePtr lane3 = create_lane_from_lane_width(-1, p2->get_reference_line(), lane_width_2, 0.05f);
  LanePtr lane4 = create_lane_from_lane_width(1, p2->get_reference_line(), lane_width_2, 0.05f);

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

  OpenDriveMapPtr map(new OpenDriveMap());
  map->add_road(r);
  map->add_road(r2);
  map->add_junction(j);

  // call
  RoadPtr ret_road = map->get_road(100);
  LaneSectionPtr ret_ls = ret_road->get_lane_sections()[0];
  Lanes ret_lane = ret_ls->get_lanes();
  auto result = ret_lane.find(7);
  Line ret_line = result->second->get_line();

  EXPECT_NEAR(bg::get<0>(ret_line.obj_[0]), 0.0f, 0.1f);
  EXPECT_NEAR(bg::get<1>(ret_line.obj_[0]), -1.0f, 0.1f);
  EXPECT_NEAR(bg::get<0>(ret_line.obj_[ret_line.obj_.size() - 1]), 10.0f, 0.1f);
  EXPECT_NEAR(bg::get<1>(ret_line.obj_[ret_line.obj_.size() - 1]), -1.0f, 0.1f);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
