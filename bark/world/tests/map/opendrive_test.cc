// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "gtest/gtest.h"

#include "bark/geometry/commons.hpp"
#include "bark/geometry/line.hpp"
#include "bark/world/opendrive/commons.hpp"
#include "bark/world/opendrive/junction.hpp"
#include "bark/world/opendrive/lane_section.hpp"
#include "bark/world/opendrive/opendrive.hpp"
#include "bark/world/opendrive/plan_view.hpp"
#include "bark/world/opendrive/road.hpp"

TEST(create_plan_view, open_drive) {
  using namespace std;
  using namespace bark::world::opendrive;
  using namespace bark::geometry;

  //! new plan view
  PlanView p;

  //! add line
  p.AddLine(Point2d(0.0f, 0.0f), 1.5, 10.0f);

  //! add arc
  p.AddArc(Point2d(0.0f, 0.0f), 0.0f, 10.0f, 0.1f);

  //! add spiral
  p.AddSpiral(Point2d(0.0f, 0.0f), 0.0f, 1.0f, 0.0f, 0.7f);

  // p.print_points();
}

TEST(lane, open_drive) {
  using namespace std;
  using namespace bark::world::opendrive;
  using namespace bark::geometry;

  //! new plan view
  PlanView p;
  XodrLaneOffset off = {1.5f, 0.0f, 0.0f, 0.0f};

  //! vertical
  p.AddLine(Point2d(0.0f, 0.0f), 1.5707, 10.0f);
  XodrLaneWidth lane_width = {0, 10.0, off};

  XodrLanePtr lane = CreateLaneFromLaneWidth(1, p.GetReferenceLine(),
                                             lane_width, 0.05f);  // left side

  lane->SetLaneType(XodrLaneType::DRIVING);
  EXPECT_EQ(lane->GetLaneType(), XodrLaneType::DRIVING);

  Line line = lane->GetLine();

  EXPECT_NEAR(bg::get<0>(line.obj_[0]), -1.5, 0.1);
  EXPECT_NEAR(bg::get<1>(line.obj_[0]), 0.0, 0.1);
  EXPECT_NEAR(bg::get<0>(line.obj_[line.obj_.size() - 1]), -1.5, 0.1);
  EXPECT_NEAR(bg::get<1>(line.obj_[line.obj_.size() - 1]), 10.0, 0.1);

  lane = CreateLaneFromLaneWidth(-1, p.GetReferenceLine(), lane_width,
                                 0.05f);  // right side

  line = lane->GetLine();

  std::cout << bg::get<1>(line.obj_[line.obj_.size() - 1]) << std::endl;

  EXPECT_NEAR(bg::get<0>(line.obj_[0]), 1.5, 0.1);
  EXPECT_NEAR(bg::get<1>(line.obj_[0]), 0.0, 0.1);
  EXPECT_NEAR(bg::get<0>(line.obj_[line.obj_.size() - 1]), 1.5, 0.1);
  EXPECT_NEAR(bg::get<1>(line.obj_[line.obj_.size() - 1]), 10.0, 0.1);

  //! new plan view
  PlanView p2;

  //! horizontal
  p2.AddLine(Point2d(0.0f, 0.0f), 0.0f, 10.0f);

  lane = CreateLaneFromLaneWidth(1, p2.GetReferenceLine(), lane_width,
                                 0.05f);  // left side

  line = lane->GetLine();

  EXPECT_NEAR(bg::get<0>(line.obj_[0]), 0.0, 0.1);
  EXPECT_NEAR(bg::get<1>(line.obj_[0]), 1.5, 0.1);
  EXPECT_NEAR(bg::get<0>(line.obj_[line.obj_.size() - 1]), 10.0, 0.1);
  EXPECT_NEAR(bg::get<1>(line.obj_[line.obj_.size() - 1]), 1.5, 0.1);

  lane = CreateLaneFromLaneWidth(-1, p2.GetReferenceLine(), lane_width,
                                 0.05f);  // right side

  line = lane->GetLine();
  EXPECT_NEAR(bg::get<0>(line.obj_[0]), 0.0, 0.1);
  EXPECT_NEAR(bg::get<1>(line.obj_[0]), -1.5, 0.1);
  EXPECT_NEAR(bg::get<0>(line.obj_[line.obj_.size() - 1]), 10.0, 0.1);
  EXPECT_NEAR(bg::get<1>(line.obj_[line.obj_.size() - 1]), -1.5, 0.1);

  // spiral and arc tests are ommitted due to their complexits -> verify with
  // plots!
}

TEST(multiple_lane_widths, open_drive) {
  using namespace bark::world::opendrive;
  using namespace bark::geometry;

  //! new plan view
  PlanView p;
  XodrLaneOffset off1 = {1.5f, 0.0f, 0.0f, 0.0f};
  XodrLaneOffset off2 = {0.0f, 0.003f, 0.0f, 0.0f};

  XodrLaneWidth lane_width1 = {0, 4.0, off1};
  XodrLaneWidth lane_width2 = {4.0, 10.0, off2};
  //! vertical
  p.AddLine(Point2d(0.0f, 0.0f), 1.5707, 10.0f);

  XodrLanePtr lane = std::make_shared<XodrLane>(1);
  bool succ = lane->append(p.GetReferenceLine(), lane_width1, 0.05f);

  Line linel1 = lane->GetLine();
  float length1 = bg::get<1>(linel1.obj_[linel1.obj_.size() - 1]);

  succ = lane->append(p.GetReferenceLine(), lane_width2, 0.05f);

  Line linel2 = lane->GetLine();
  float length2 = bg::get<1>(linel2.obj_[linel2.obj_.size() - 1]);

  EXPECT_NEAR(lane_width1.s_end, length1, 0.1);
  EXPECT_NEAR(lane_width2.s_start, length1, 0.1);
  EXPECT_NEAR(lane_width2.s_end, length2, 0.1);
  EXPECT_TRUE(length1 < length2);
}

TEST(road, open_drive) {
  using namespace std;
  using namespace bark::world::opendrive;
  using namespace bark::geometry;

  //! new plan view
  PlanViewPtr p(new PlanView());
  p->AddLine(Point2d(0.0f, 0.0f), 1.5707, 10.0f);

  XodrRoadLinkInfo pre;
  pre.id_ = 2;
  pre.type_ = "road";

  XodrRoadLinkInfo suc;
  suc.id_ = 1;
  suc.type_ = "road";

  //! XodrRoad-Link
  XodrRoadLink l;  // can either link to another road or to a junction
  l.SetPredecessor(pre);
  l.SetSuccessor(suc);

  //! XodrLane-Section 1
  XodrLaneSectionPtr ls(new XodrLaneSection(0.0));

  //! XodrLane-Section 2
  XodrLaneSectionPtr ls2(new XodrLaneSection(10.0));

  //! XodrLane
  XodrLaneOffset off = {1.0f, 0.0f, 0.0f, 0.0f};
  XodrLaneWidth lane_width = {0, 1, off};
  XodrLanePtr lane =
      CreateLaneFromLaneWidth(-1, p->GetReferenceLine(), lane_width, 0.05f);
  XodrLanePtr lane2 =
      CreateLaneFromLaneWidth(1, p->GetReferenceLine(), lane_width, 0.05f);

  ls->AddLane(lane);
  ls2->AddLane(lane2);

  //! new road
  XodrRoad r("highway", 1);
  r.SetPlanView(p);
  r.SetLink(l);
  r.AddLaneSection(ls);
}

TEST(junction, open_drive) {
  using namespace std;
  using namespace bark::world::opendrive;
  using namespace bark::geometry;

  Junction j("kreuz", 1);

  Connection con;
  con.incoming_road_ = 100;
  con.connecting_road_ = 200;
  con.id_ = 1;

  XodrLaneLink l1 = {1, -1};
  XodrLaneLink l2 = {2, 2};

  con.AddLaneLink(l1);
  con.AddLaneLink(l2);

  j.AddConnection(con);
}

TEST(map, open_drive) {
  using namespace std;
  using namespace bark::world::opendrive;
  using namespace bark::geometry;

  //! ROAD 1
  PlanViewPtr p(new PlanView());
  p->AddLine(Point2d(0.0f, 0.0f), 0.0f, 10.0f);

  //! XodrRoad-Link

  XodrRoadLinkInfo pre;
  pre.id_ = 1;
  pre.type_ = "road";

  XodrRoadLink l;  // can either link to another road or to a junction
  l.SetPredecessor(pre);
  l.SetSuccessor(pre);
  //! XodrLane-Section 1
  XodrLaneSectionPtr ls(new XodrLaneSection(0.0));

  //! XodrLane-Section 2
  XodrLaneSectionPtr ls2(new XodrLaneSection(10.0));

  //! XodrLane
  XodrLaneOffset off = {1.0f, 0.0f, 0.0f, 0.0f};
  XodrLaneWidth lane_width_1 = {0, 10.0, off};

  XodrLanePtr lane =
      CreateLaneFromLaneWidth(-1, p->GetReferenceLine(), lane_width_1, 0.05f);
  XodrLanePtr lane2 =
      CreateLaneFromLaneWidth(1, p->GetReferenceLine(), lane_width_1, 0.05f);

  XodrRoadMark rm{roadmark::XodrRoadMarkType::SOLID,
                  roadmark::XodrRoadMarkColor::STANDARD, 0.1};

  std::cout << print(rm) << std::endl;
  lane2->SetRoadMark(rm);

  ls->AddLane(lane);
  ls->AddLane(lane2);
  ls2->AddLane(lane);
  ls2->AddLane(lane2);

  XodrRoadPtr r(new XodrRoad("highway", 100));
  r->SetPlanView(p);
  r->SetLink(l);
  r->AddLaneSection(ls);

  //! ROAD 2
  PlanViewPtr p2(new PlanView());
  p2->AddLine(Point2d(0.0f, 0.0f), 0.0f, 10.0f);

  //! XodrRoad-Link
  XodrRoadLink l2 = {};  // can either link to another road or to a junction

  //! XodrLane-Section 1
  XodrLaneSectionPtr ls3(new XodrLaneSection(0.0));

  //! XodrLane
  XodrLaneOffset off2 = {1.0f, 0.0f, 0.0f, 0.0f};
  XodrLaneWidth lane_width_2 = {0, 10.0, off2};

  XodrLanePtr lane3 =
      CreateLaneFromLaneWidth(-1, p2->GetReferenceLine(), lane_width_2, 0.05f);
  XodrLanePtr lane4 =
      CreateLaneFromLaneWidth(1, p2->GetReferenceLine(), lane_width_2, 0.05f);

  ls3->AddLane(lane3);
  ls3->AddLane(lane4);

  //! new road
  XodrRoadPtr r2(new XodrRoad("highway", 200));
  r2->SetPlanView(p2);
  r2->SetLink(l2);
  r2->AddLaneSection(ls3);

  // connect roads
  JunctionPtr j(new Junction("kreuz", 1));
  Connection con;
  con.incoming_road_ = 100;
  con.connecting_road_ = 200;
  con.id_ = 1;

  XodrLaneLink link_1 = {-1, -1};
  XodrLaneLink link_2 = {1, 1};

  con.AddLaneLink(link_1);
  con.AddLaneLink(link_2);
  j->AddConnection(con);

  OpenDriveMapPtr map(new OpenDriveMap());
  map->AddRoad(r);
  map->AddRoad(r2);
  map->AddJunction(j);

  // call
  XodrRoadPtr ret_road = map->GetRoad(100);
  XodrLaneSectionPtr ret_ls = ret_road->GetLaneSections()[0];
  XodrLanes ret_lanes = ret_ls->GetLanes();

  EXPECT_GT(ret_lanes.size(), 0);

  for (auto const& rl : ret_lanes)
    std::cout << "lane ids are " << rl.first << " , ";

  auto result = ret_lanes.find(8);

  EXPECT_TRUE(result != ret_lanes.end());

  std::cout << "Found lane  " << result->first << " "
            << print(*(result->second)) << '\n';
  Line ret_line = result->second->GetLine();

  EXPECT_NEAR(bg::get<0>(ret_line.obj_[0]), 0.0f, 0.1f);
  EXPECT_NEAR(bg::get<1>(ret_line.obj_[0]), -1.0f, 0.1f);
  EXPECT_NEAR(bg::get<0>(ret_line.obj_[ret_line.obj_.size() - 1]), 10.0f, 0.1f);
  EXPECT_NEAR(bg::get<1>(ret_line.obj_[ret_line.obj_.size() - 1]), -1.0f, 0.1f);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
