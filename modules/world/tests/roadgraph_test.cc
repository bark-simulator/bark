// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include <utility>
#include "gtest/gtest.h"
#include "modules/world/map/roadgraph.hpp"
#include "modules/world/opendrive/opendrive.hpp"


TEST(roadgraph, road_creation)
{
  using namespace modules::world::map;
  Roadgraph r;
  
  RoadId rid = 0;
  LanePtr lane_0(new Lane());
  LanePtr lane_plus1(new Lane());
  LanePtr lane_plus2(new Lane());
  LanePtr lane_minus1(new Lane());

  LaneId l0 = r.add_lane(rid, lane_0);
  LaneId l1 = r.add_lane(rid, lane_plus1);
  LaneId l2 = r.add_lane(rid, lane_plus2);
  LaneId lm1 = r.add_lane(rid, lane_minus1);

  r.add_inner_neighbor(lm1, l0);
  r.add_outer_neighbor(lm1, l0);
  r.add_inner_neighbor(l0, l1);
  r.add_outer_neighbor(l0, l1);
  r.add_inner_neighbor(l1, l2);
  r.add_outer_neighbor(l1, l2);

  int num_vertices = static_cast<int>(boost::num_vertices(r.get_lane_graph()));
  int num_edges = static_cast<int>(boost::num_edges(r.get_lane_graph()));

  r.print_graph("road_creation.dot");

  ASSERT_EQ(4, num_vertices);
  ASSERT_EQ(6, num_edges);

}

TEST(roadgraph, road_adjacency)
{
  using namespace modules::world::map;
  Roadgraph r;


  RoadId rid0 = 0;
  LanePtr lane_00(new Lane());
  LanePtr lane_plus10(new Lane());
  LanePtr lane_minus10(new Lane());

  LaneId l00 = r.add_lane(rid0, lane_00);
  LaneId l10 = r.add_lane(rid0, lane_plus10);
  LaneId lm10 = r.add_lane(rid0, lane_minus10);

  r.add_inner_neighbor(lm10, l00);
  r.add_outer_neighbor(lm10, l00);
  r.add_inner_neighbor(l00, l10);
  r.add_outer_neighbor(l00, l10);

  RoadId rid1 = 1;
  LanePtr lane_01(new Lane());
  LanePtr lane_plus11(new Lane());
  LanePtr lane_minus11(new Lane());

  LaneId l01 = r.add_lane(rid1, lane_01);
  LaneId l11 = r.add_lane(rid1, lane_plus11);
  LaneId lm11 = r.add_lane(rid1, lane_minus11);

  r.add_inner_neighbor(lm11, l01);
  r.add_outer_neighbor(lm11, l01);
  r.add_inner_neighbor(l01, l11);
  r.add_outer_neighbor(l01, l11);

  r.add_successor(l10, l11);
  r.add_successor(lm11, lm10);

  int num_vertices = static_cast<int>(boost::num_vertices(r.get_lane_graph()));
  int num_edges = static_cast<int>(boost::num_edges(r.get_lane_graph()));

  r.print_graph("road_successors.dot");

  ASSERT_EQ(6, num_vertices);
  ASSERT_EQ(10, num_edges);

}

TEST(roadgraph, road_adjacency_simple)
{
  using namespace modules::world::map;
  Roadgraph r;

  RoadId rid0 = 0;
  LanePtr lane_00(new Lane());
  LaneId l00 = r.add_lane(rid0, lane_00);

  RoadId rid1 = 1;
  LanePtr lane_01(new Lane());
  LaneId l01 = r.add_lane(rid1, lane_01);

  r.add_successor(l00, l01);


  int num_vertices = static_cast<int>(boost::num_vertices(r.get_lane_graph()));
  int num_edges = static_cast<int>(boost::num_edges(r.get_lane_graph()));

  r.print_graph("road_successors_simple.dot");

  ASSERT_EQ(2, num_vertices);
  ASSERT_EQ(1, num_edges);

}


TEST(roadgraph, get_successor_lane_test)
{
  using namespace modules::world::map;
  Roadgraph r;

  RoadId rid0 = 0;
  LanePtr lane_00(new Lane());
  LanePtr lane_10(new Lane());
  LaneId l00 = r.add_lane(rid0, lane_00);
  LaneId l10 = r.add_lane(rid0, lane_10);

  RoadId rid1 = 1;
  LanePtr lane_01(new Lane());
  LaneId l01 = r.add_lane(rid1, lane_01);

  r.add_successor(l00, l01);
  r.add_inner_neighbor(l00, l10);

  std::vector<LaneId> suc = r.get_successor_lanes(l00);
  ASSERT_TRUE(suc.size() == 1);
  ASSERT_TRUE(suc[0] == l01);
}

TEST(roadgraph, get_predecessor_lane_test)
{
  using namespace modules::world::map;
  Roadgraph r;

  RoadId rid0 = 0;
  LanePtr lane_00(new Lane());
  LanePtr lane_10(new Lane());
  LaneId l00 = r.add_lane(rid0, lane_00);
  LaneId l10 = r.add_lane(rid0, lane_10);

  RoadId rid1 = 1;
  LanePtr lane_01(new Lane());
  LaneId l01 = r.add_lane(rid1, lane_01);

  r.add_successor(l00, l01);
  r.add_inner_neighbor(l00, l10);

  std::vector<LaneId> pre = r.get_predecessor_lanes(l01);
  ASSERT_TRUE(pre.size() == 1);
  ASSERT_TRUE(pre[0] == l00);
}


TEST(roadgraph, get_inner_neighbor_test)
{
  using namespace modules::world::map;
  Roadgraph r;

  RoadId rid0 = 0;
  LanePtr lane_00(new Lane(1));
  LanePtr lane_10(new Lane(-1));
  LaneId l00 = r.add_lane(rid0, lane_00);
  LaneId l10 = r.add_lane(rid0, lane_10);

  RoadId rid1 = 1;
  LanePtr lane_01(new Lane(1));
  LaneId l01 = r.add_lane(rid1, lane_01);

  r.add_successor(l00, l01);
  r.add_inner_neighbor(l00, l10);

  std::pair<LaneId, bool> in = r.get_inner_neighbor(l10);
  ASSERT_TRUE(in.first == l00);
  ASSERT_TRUE(in.second);
}

TEST(roadgraph, get_outer_neighbor_test)
{
  using namespace modules::world::map;
  Roadgraph r;

  RoadId rid0 = 0;
  LanePtr lane_00(new Lane(1));
  LanePtr lane_10(new Lane(-1));
  LaneId l00 = r.add_lane(rid0, lane_00);
  LaneId l10 = r.add_lane(rid0, lane_10);

  r.add_inner_neighbor(l00, l10);
  r.add_outer_neighbor(l00, l10);

  std::pair<LaneId, bool> in = r.get_outer_neighbor(l00);
  ASSERT_TRUE(in.first == l10);
  ASSERT_TRUE(in.second);
}

TEST(roadgraph, get_inner_neighbor_test_planview)
{
  using namespace modules::world::map;
  Roadgraph r;

  RoadId rid0 = 0;
  LanePtr lane_m1(new Lane(-1));
  LanePtr lane_0(new Lane(0));
  LanePtr lane_1(new Lane(-1));
  LaneId l0 = r.add_lane(rid0, lane_0);
  LaneId l1 = r.add_lane(rid0, lane_1);
  LaneId lm1 = r.add_lane(rid0, lane_m1);

  r.add_inner_neighbor(l0, l1);
  r.add_inner_neighbor(l0, lm1);
  r.add_outer_neighbor(l0, l1);
  r.add_outer_neighbor(l0, lm1);

  std::pair<LaneId, bool> in = r.get_inner_neighbor(l1);
  ASSERT_TRUE(in.first == l0);
  ASSERT_TRUE(in.second);

  std::pair<LaneId, bool> in2 = r.get_inner_neighbor(lm1);
  ASSERT_TRUE(in2.first == l0);
  ASSERT_TRUE(in2.second);
}

TEST(roadgraph, find_path_test)
{
  using namespace modules::world::map;
  Roadgraph r;

  LanePtr lane_0(new Lane()); lane_0->set_id(10); lane_0->set_lane_type(LaneType::DRIVING);
  LanePtr lane_1(new Lane()); lane_1->set_id(20); lane_1->set_lane_type(LaneType::DRIVING);
  LanePtr lane_2(new Lane()); lane_2->set_id(30); lane_2->set_lane_type(LaneType::DRIVING);
  LanePtr lane_3(new Lane()); lane_3->set_id(40); lane_3->set_lane_type(LaneType::DRIVING);
  LanePtr lane_4(new Lane()); lane_4->set_id(50); lane_4->set_lane_type(LaneType::DRIVING);
  LanePtr lane_5(new Lane()); lane_5->set_id(60); lane_5->set_lane_type(LaneType::DRIVING);
  LaneId l0 = r.add_lane(0, lane_0);
  LaneId l1 = r.add_lane(1, lane_1);
  LaneId l2 = r.add_lane(2, lane_2);
  LaneId l3 = r.add_lane(3, lane_3);
  LaneId l4 = r.add_lane(4, lane_4);
  LaneId l5 = r.add_lane(5, lane_5);

  r.add_successor(l0, l1);
  r.add_successor(l1, l2);
  r.add_successor(l2, l3);
  r.add_successor(l3, l4);
  r.add_successor(l0, l5);
  r.add_successor(l5, l3);


  std::vector<LaneId> path = r.find_path(l0,l4);
  ASSERT_EQ(4, path.size());
  ASSERT_EQ(l0, path[0]);
  ASSERT_EQ(path[1], l5);
  ASSERT_EQ(path[2], l3);
  ASSERT_EQ(path[3], l4);

  path.clear();
  path = r.find_path(l0,l2);
  ASSERT_EQ(3, path.size());
  ASSERT_EQ(path[0], l0);
  ASSERT_EQ(path[1], l1);
  ASSERT_EQ(path[2], l2);
  
}

TEST(roadgraph, find_path_in_unconnected_graph_test)
{
  using namespace modules::world::map;
  Roadgraph r;

  LanePtr lane_0(new Lane()); lane_0->set_id(10); lane_0->set_lane_type(LaneType::DRIVING);
  LanePtr lane_1(new Lane()); lane_1->set_id(20); lane_1->set_lane_type(LaneType::DRIVING);
  LanePtr lane_2(new Lane()); lane_2->set_id(30); lane_2->set_lane_type(LaneType::DRIVING);
  LanePtr lane_3(new Lane()); lane_3->set_id(40); lane_3->set_lane_type(LaneType::DRIVING);
  LanePtr lane_4(new Lane()); lane_4->set_id(50); lane_4->set_lane_type(LaneType::DRIVING);
  LaneId l0 = r.add_lane(0, lane_0);
  LaneId l1 = r.add_lane(1, lane_1);
  LaneId l2 = r.add_lane(2, lane_2);
  LaneId l3 = r.add_lane(3, lane_3);
  LaneId l4 = r.add_lane(4, lane_4);

  r.add_successor(l0, l1);
  r.add_successor(l1, l2);
  // r.add_successor(l2, l3); REMOVING CONNECTION, thus not a valid path available
  r.add_successor(l3, l4);

  std::vector<LaneId> path = r.find_path(l0,l4);
  ASSERT_EQ(0, path.size());
 
}


TEST(roadgraph, find_drivable_path_test)
{
  using namespace modules::world::map;
  Roadgraph r;

  LanePtr lane_0(new Lane()); lane_0->set_id(10); lane_0->set_lane_type(LaneType::DRIVING);
  LanePtr lane_1(new Lane()); lane_1->set_id(20); lane_1->set_lane_type(LaneType::DRIVING);
  LanePtr lane_2(new Lane()); lane_2->set_id(30); lane_2->set_lane_type(LaneType::DRIVING);
  LanePtr lane_3(new Lane()); lane_3->set_id(40); lane_3->set_lane_type(LaneType::DRIVING);
  LanePtr lane_4(new Lane()); lane_4->set_id(50); lane_4->set_lane_type(LaneType::DRIVING);
  LanePtr lane_5(new Lane()); lane_5->set_id(60); lane_5->set_lane_type(LaneType::SIDEWALK);
  LaneId l0 = r.add_lane(0, lane_0); 
  LaneId l1 = r.add_lane(1, lane_1);
  LaneId l2 = r.add_lane(2, lane_2);
  LaneId l3 = r.add_lane(3, lane_3);
  LaneId l4 = r.add_lane(4, lane_4);
  LaneId l5 = r.add_lane(5, lane_5);

  r.add_successor(l0, l1);
  r.add_successor(l1, l2);
  r.add_successor(l2, l3);
  r.add_successor(l3, l4);
  r.add_successor(l0, l5);
  r.add_successor(l5, l3);

  std::vector<LaneId> path = r.find_path(l0,l4);
  ASSERT_EQ(5, path.size());
  ASSERT_EQ(path[0], l0);
  ASSERT_EQ(path[1], l1);
  ASSERT_EQ(path[2], l2);
  ASSERT_EQ(path[3], l3);
  ASSERT_EQ(path[4], l4);
}

TEST(roadgraph, find_drivable_path_in_unconnected_graph_test)
{
  using namespace modules::world::map;
  Roadgraph r;

  LanePtr lane_0(new Lane()); lane_0->set_id(10); lane_0->set_lane_type(LaneType::DRIVING);
  LanePtr lane_1(new Lane()); lane_1->set_id(20); lane_1->set_lane_type(LaneType::DRIVING);
  LanePtr lane_2(new Lane()); lane_2->set_id(30); lane_2->set_lane_type(LaneType::SIDEWALK);
  LanePtr lane_3(new Lane()); lane_3->set_id(40); lane_3->set_lane_type(LaneType::DRIVING);
  LaneId l0 = r.add_lane(0, lane_0); 
  LaneId l1 = r.add_lane(1, lane_1);
  LaneId l2 = r.add_lane(2, lane_2);
  LaneId l3 = r.add_lane(3, lane_3);

  r.add_successor(l0, l1);
  r.add_successor(l1, l2);
  r.add_successor(l2, l3);

  std::vector<LaneId> path = r.find_path(l0,l3);
  ASSERT_EQ(0, path.size());
}

TEST(roadgraph, find_drivable_path_invalid_final_vertex_test)
{
  using namespace modules::world::map;
  Roadgraph r;

  LanePtr lane_0(new Lane()); lane_0->set_id(10); lane_0->set_lane_type(LaneType::DRIVING);
  LanePtr lane_1(new Lane()); lane_1->set_id(20); lane_1->set_lane_type(LaneType::DRIVING);
  LanePtr lane_2(new Lane()); lane_2->set_id(30); lane_2->set_lane_type(LaneType::DRIVING);
  LanePtr lane_3(new Lane()); lane_3->set_id(40); lane_3->set_lane_type(LaneType::SIDEWALK);
  LaneId l0 = r.add_lane(0, lane_0); 
  LaneId l1 = r.add_lane(1, lane_1);
  LaneId l2 = r.add_lane(2, lane_2);
  LaneId l3 = r.add_lane(3, lane_3);

  r.add_successor(l0, l1);
  r.add_successor(l1, l2);
  r.add_successor(l2, l3);

  std::vector<LaneId> path = r.find_path(l0,l3);
  ASSERT_EQ(0, path.size());
}

TEST(roadgraph, get_driving_corridor_test)
{
  using namespace modules::world::map;
  Roadgraph r;

  RoadId rid0 = 0;
  LanePtr lane_00(new Lane()); 
  LanePtr lane_10(new Lane());
  LaneId l00 = r.add_lane(rid0, lane_00);
  LaneId l10 = r.add_lane(rid0, lane_10);

  RoadId rid1 = 1;
  LanePtr lane_01(new Lane());
  LanePtr lane_11(new Lane());
  LaneId l01 = r.add_lane(rid1, lane_01);
  LaneId l11 = r.add_lane(rid1, lane_11);

  r.add_successor(l10, l11);
  r.add_inner_neighbor(l00, l10);
  r.add_inner_neighbor(l01, l11);

  // TODO(@fortiss): add new assertions as this has moved to MapInterface
  /*
  std::pair< std::vector<LanePtr>, std::vector<LanePtr> > horizon = r.ComputeLaneBoundariesHorizon(l10, l11);

  ASSERT_TRUE(horizon.first.size()==2);
  ASSERT_TRUE(horizon.second.size()==2);
  */
}

TEST(roadgraph, generate_roadgraph_neighbours)
{
  using namespace modules::geometry;
  using namespace modules::world::opendrive;
  using namespace modules::world::map;

  OpenDriveMapPtr open_drive_map(new OpenDriveMap());
    //! ROAD 1
  PlanViewPtr p(new PlanView());
  p->add_line(Point2d(0.0f, 0.0f), 0.0f, 10.0f);

  //! Lane-Section 1
  LaneSectionPtr ls(new LaneSection(0.0));

  //! Lane
  LaneOffset off = {1.0f, 0.0f, 0.0f, 0.0f};
  LaneWidth lane_width_1 = {0, 10, off};
  LanePtr lane1 = create_lane_from_lane_width(-1, p->get_reference_line(), lane_width_1, 0.05);
  LanePtr lane2 = create_lane_from_lane_width(1, p->get_reference_line(), lane_width_1, 0.05);
  LanePtr lane3 = create_lane_from_lane_width(2, lane2->get_line(), lane_width_1, 0.05);

  ls->add_lane(lane1);
  ls->add_lane(lane2);
  ls->add_lane(lane3);

  RoadPtr r(new Road("highway", 100));
  r->set_plan_view(p);
  r->add_lane_section(ls);

  open_drive_map->add_road(r);

  Roadgraph rg;
  rg.Generate(open_drive_map);

  std::pair<LaneId, bool> in1 = rg.get_inner_neighbor(lane1->get_id());
  // There is no inner neighbor for lane1, as lane1 and lane2 are left and right of the planview
  ASSERT_FALSE(in1.second);

  std::pair<LaneId, bool> out2 = rg.get_outer_neighbor(lane2->get_id());
  ASSERT_TRUE(out2.second);
  ASSERT_TRUE(out2.first == lane3->get_id());

}


TEST(roadgraph, generate_roadgraph_successors)
{
  using namespace modules::geometry;
  using namespace modules::world::opendrive;
  using namespace modules::world::map;

  OpenDriveMapPtr open_drive_map(new OpenDriveMap());
  //! ROAD 1
  PlanViewPtr p(new PlanView());
  p->add_line(Point2d(0.0f, 0.0f), 0.0f, 10.0f);

  //! Lane-Section 1
  LaneSectionPtr ls(new LaneSection(0.0));
  
  //! Lane
  LaneOffset off = {1.0f, 0.0f, 0.0f, 0.0f};
  LaneWidth lane_width_1 = {0, 10, off};
  LanePtr lane1 = create_lane_from_lane_width(1, p->get_reference_line(), lane_width_1, 0.05);
  LaneLink ll1;
  ll1.from_position = 1;
  ll1.to_position = 1;
  lane1->set_link(ll1);
  ls->add_lane(lane1);

  RoadPtr r(new Road("highway", 100));
  r->set_plan_view(p);
  r->add_lane_section(ls);

  //! ROAD 2
  PlanViewPtr p2(new PlanView());
  p2->add_line(Point2d(10.0f, 0.0f), 0.0f, 20.0f);

  //! Lane-Section 2
  LaneSectionPtr ls2(new LaneSection(0.0));
  
  //! Lane
  LaneOffset off2 = {1.0f, 0.0f, 0.0f, 0.0f};
  LaneWidth lane_width_2 = {0, 10, off};
  LanePtr lane2 = create_lane_from_lane_width(1, p2->get_reference_line(), lane_width_2, 0.05);
  LaneLink ll2;
  ll2.from_position = 1;
  ll2.to_position = 1;
  lane2->set_link(ll2);
  ls2->add_lane(lane2);

  RoadPtr r2(new Road("highway", 101));
  r2->set_plan_view(p2);
  r2->add_lane_section(ls2);
  
  RoadLinkInfo rli_succ(r2->get_id(), "road");
  RoadLink rl;
  rl.set_successor(rli_succ);
  r->set_link(rl);

  open_drive_map->add_road(r);
  open_drive_map->add_road(r2);

  Roadgraph rg;
  rg.Generate(open_drive_map);

  std::vector<LaneId> suc = rg.get_successor_lanes(lane1->get_id());
  std::cout << "suc: " << suc.size() << std::endl;
  ASSERT_TRUE(suc.size() == 1);
  ASSERT_TRUE(suc[0] == lane2->get_id());
  
  auto all_ids = rg.get_all_laneids();
  ASSERT_TRUE(all_ids.size() == 2);
}
