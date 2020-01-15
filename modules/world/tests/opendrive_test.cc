// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "gtest/gtest.h"

#include "modules/geometry/commons.hpp"
#include "modules/geometry/line.hpp"
#include "modules/world/opendrive/commons.hpp"
#include "modules/world/opendrive/junction.hpp"
#include "modules/world/opendrive/lane_section.hpp"
#include "modules/world/opendrive/opendrive.hpp"
#include "modules/world/opendrive/plan_view.hpp"
#include "modules/world/opendrive/road.hpp"

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

  // p.print_points();
}

TEST(lane, open_drive) {
  using namespace std;
  using namespace modules::world::opendrive;
  using namespace modules::geometry;

  //! new plan view
  PlanView p;
  XodrLaneOffset off = {1.5f, 0.0f, 0.0f, 0.0f};

  //! vertical
  p.add_line(Point2d(0.0f, 0.0f), 1.5707, 10.0f);
  XodrLaneWidth lane_width = {0, 10.0, off};

  XodrLanePtr lane = create_lane_from_lane_width(1, p.get_reference_line(),
                                             lane_width, 0.05f);  // left side

  lane->set_lane_type(XodrLaneType::DRIVING);
  EXPECT_EQ(lane->get_lane_type(), XodrLaneType::DRIVING);

  Line line = lane->get_line();

  EXPECT_NEAR(bg::get<0>(line.obj_[0]), -1.5, 0.1);
  EXPECT_NEAR(bg::get<1>(line.obj_[0]), 0.0, 0.1);
  EXPECT_NEAR(bg::get<0>(line.obj_[line.obj_.size() - 1]), -1.5, 0.1);
  EXPECT_NEAR(bg::get<1>(line.obj_[line.obj_.size() - 1]), 10.0, 0.1);

  lane = create_lane_from_lane_width(-1, p.get_reference_line(), lane_width,
                                     0.05f);  // right side

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

  lane = create_lane_from_lane_width(1, p2.get_reference_line(), lane_width,
                                     0.05f);  // left side

  line = lane->get_line();

  EXPECT_NEAR(bg::get<0>(line.obj_[0]), 0.0, 0.1);
  EXPECT_NEAR(bg::get<1>(line.obj_[0]), 1.5, 0.1);
  EXPECT_NEAR(bg::get<0>(line.obj_[line.obj_.size() - 1]), 10.0, 0.1);
  EXPECT_NEAR(bg::get<1>(line.obj_[line.obj_.size() - 1]), 1.5, 0.1);

  lane = create_lane_from_lane_width(-1, p2.get_reference_line(), lane_width,
                                     0.05f);  // right side

  line = lane->get_line();
  EXPECT_NEAR(bg::get<0>(line.obj_[0]), 0.0, 0.1);
  EXPECT_NEAR(bg::get<1>(line.obj_[0]), -1.5, 0.1);
  EXPECT_NEAR(bg::get<0>(line.obj_[line.obj_.size() - 1]), 10.0, 0.1);
  EXPECT_NEAR(bg::get<1>(line.obj_[line.obj_.size() - 1]), -1.5, 0.1);

  // spiral and arc tests are ommitted due to their complexits -> verify with
  // plots!
}

TEST(multiple_lane_widths, open_drive) {
  using namespace modules::world::opendrive;
  using namespace modules::geometry;

  //! new plan view
  PlanView p;
  XodrLaneOffset off1 = {1.5f, 0.0f, 0.0f, 0.0f};
  XodrLaneOffset off2 = {0.0f, 0.003f, 0.0f, 0.0f};

  XodrLaneWidth lane_width1 = {0, 4.0, off1};
  XodrLaneWidth lane_width2 = {4.0, 10.0, off2};
  //! vertical
  p.add_line(Point2d(0.0f, 0.0f), 1.5707, 10.0f);

  XodrLanePtr lane = std::make_shared<XodrLane>(1);
  bool succ = lane->append(p.get_reference_line(), lane_width1, 0.05f);

  Line linel1 = lane->get_line();
  float length1 = bg::get<1>(linel1.obj_[linel1.obj_.size() - 1]);

  succ = lane->append(p.get_reference_line(), lane_width2, 0.05f);

  Line linel2 = lane->get_line();
  float length2 = bg::get<1>(linel2.obj_[linel2.obj_.size() - 1]);

  EXPECT_NEAR(lane_width1.s_end, length1, 0.1);
  EXPECT_NEAR(lane_width2.s_start, length1, 0.1);

  EXPECT_NEAR(lane_width2.s_end, length2, 0.1);

  EXPECT_TRUE(length1 < length2);
}

TEST(road, open_drive) {
  using namespace std;
  using namespace modules::world::opendrive;
  using namespace modules::geometry;

  //! new plan view
  PlanViewPtr p(new PlanView());
  p->add_line(Point2d(0.0f, 0.0f), 1.5707, 10.0f);

  XodrRoadLinkInfo pre;
  pre.id_ = 2;
  pre.type_ = "road";

  XodrRoadLinkInfo suc;
  suc.id_ = 1;
  suc.type_ = "road";

  //! XodrRoad-Link
  XodrRoadLink l;  // can either link to another road or to a junction
  l.set_predecessor(pre);
  l.set_successor(suc);

  //! XodrLane-Section 1
  XodrLaneSectionPtr ls(new XodrLaneSection(0.0));

  //! XodrLane-Section 2
  XodrLaneSectionPtr ls2(new XodrLaneSection(10.0));

  //! XodrLane
  XodrLaneOffset off = {1.0f, 0.0f, 0.0f, 0.0f};
  XodrLaneWidth lane_width = {0, 1, off};
  XodrLanePtr lane = create_lane_from_lane_width(-1, p->get_reference_line(),
                                             lane_width, 0.05f);
  XodrLanePtr lane2 = create_lane_from_lane_width(1, p->get_reference_line(),
                                              lane_width, 0.05f);

  ls->add_lane(lane);
  ls2->add_lane(lane2);

  //! new road
  XodrRoad r("highway", 1);
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

  XodrLaneLink l1 = {1, -1};
  XodrLaneLink l2 = {2, 2};

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

  //! XodrRoad-Link

  XodrRoadLinkInfo pre;
  pre.id_ = 1;
  pre.type_ = "road";

  XodrRoadLink l;  // can either link to another road or to a junction
  l.set_predecessor(pre);
  l.set_successor(pre);
  //! XodrLane-Section 1
  XodrLaneSectionPtr ls(new XodrLaneSection(0.0));

  //! XodrLane-Section 2
  XodrLaneSectionPtr ls2(new XodrLaneSection(10.0));

  //! XodrLane
  XodrLaneOffset off = {1.0f, 0.0f, 0.0f, 0.0f};
  XodrLaneWidth lane_width_1 = {0, 10.0, off};

  XodrLanePtr lane = create_lane_from_lane_width(-1, p->get_reference_line(),
                                             lane_width_1, 0.05f);
  XodrLanePtr lane2 = create_lane_from_lane_width(1, p->get_reference_line(),
                                              lane_width_1, 0.05f);

  XodrRoadMark rm{roadmark::XodrRoadMarkType::SOLID, roadmark::XodrRoadMarkColor::STANDARD,
              0.1};

  std::cout << print(rm) << std::endl;
  lane2->set_road_mark(rm);

  ls->add_lane(lane);
  ls->add_lane(lane2);
  ls2->add_lane(lane);
  ls2->add_lane(lane2);

  XodrRoadPtr r(new XodrRoad("highway", 100));
  r->set_plan_view(p);
  r->set_link(l);
  r->add_lane_section(ls);

  //! ROAD 2
  PlanViewPtr p2(new PlanView());
  p2->add_line(Point2d(0.0f, 0.0f), 0.0f, 10.0f);

  //! XodrRoad-Link
  XodrRoadLink l2 = {};  // can either link to another road or to a junction

  //! XodrLane-Section 1
  XodrLaneSectionPtr ls3(new XodrLaneSection(0.0));

  //! XodrLane
  XodrLaneOffset off2 = {1.0f, 0.0f, 0.0f, 0.0f};
  XodrLaneWidth lane_width_2 = {0, 10.0, off2};

  XodrLanePtr lane3 = create_lane_from_lane_width(-1, p2->get_reference_line(),
                                              lane_width_2, 0.05f);
  XodrLanePtr lane4 = create_lane_from_lane_width(1, p2->get_reference_line(),
                                              lane_width_2, 0.05f);

  ls3->add_lane(lane3);
  ls3->add_lane(lane4);

  //! new road
  XodrRoadPtr r2(new XodrRoad("highway", 200));
  r2->set_plan_view(p2);
  r2->set_link(l2);
  r2->add_lane_section(ls3);

  // connect roads
  JunctionPtr j(new Junction("kreuz", 1));
  Connection con;
  con.incoming_road_ = 100;
  con.connecting_road_ = 200;
  con.id_ = 1;

  XodrLaneLink link_1 = {-1, -1};
  XodrLaneLink link_2 = {1, 1};

  con.add_lane_link(link_1);
  con.add_lane_link(link_2);
  j->add_connection(con);

  OpenDriveMapPtr map(new OpenDriveMap());
  map->add_road(r);
  map->add_road(r2);
  map->add_junction(j);

  // call
  XodrRoadPtr ret_road = map->get_road(100);
  XodrLaneSectionPtr ret_ls = ret_road->get_lane_sections()[0];
  XodrLanes ret_lanes = ret_ls->get_lanes();
  
  EXPECT_GT(ret_lanes.size(), 0);

  for (auto const& rl : ret_lanes)
    std::cout << "lane ids are " << rl.first << " , ";

  auto result = ret_lanes.find(8);

  EXPECT_TRUE(result != ret_lanes.end());

  std::cout << "Found lane  " << result->first << " "
            << print(*(result->second)) << '\n';
  Line ret_line = result->second->get_line();

  EXPECT_NEAR(bg::get<0>(ret_line.obj_[0]), 0.0f, 0.1f);
  EXPECT_NEAR(bg::get<1>(ret_line.obj_[0]), -1.0f, 0.1f);
  EXPECT_NEAR(bg::get<0>(ret_line.obj_[ret_line.obj_.size() - 1]), 10.0f, 0.1f);
  EXPECT_NEAR(bg::get<1>(ret_line.obj_[ret_line.obj_.size() - 1]), -1.0f, 0.1f);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
