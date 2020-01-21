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
  
  XodrRoadId rid = 0;
  XodrLanePtr lane_0(new XodrLane());
  XodrLanePtr lane_plus1(new XodrLane());
  XodrLanePtr lane_plus2(new XodrLane());
  XodrLanePtr lane_minus1(new XodrLane());

  XodrLaneId l0 = r.add_lane(rid, lane_0);
  XodrLaneId l1 = r.add_lane(rid, lane_plus1);
  XodrLaneId l2 = r.add_lane(rid, lane_plus2);
  XodrLaneId lm1 = r.add_lane(rid, lane_minus1);

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


  XodrRoadId rid0 = 0;
  XodrLanePtr lane_00(new XodrLane(0));
  XodrLanePtr lane_plus10(new XodrLane(1));
  XodrLanePtr lane_minus10(new XodrLane(-1));

  XodrLaneId l00 = r.add_lane(rid0, lane_00);
  XodrLaneId l10 = r.add_lane(rid0, lane_plus10);
  XodrLaneId lm10 = r.add_lane(rid0, lane_minus10);

  r.add_inner_neighbor(lm10, l00);
  r.add_outer_neighbor(lm10, l00);
  r.add_inner_neighbor(l00, l10);
  r.add_outer_neighbor(l00, l10);

  XodrRoadId rid1 = 1;
  XodrLanePtr lane_01(new XodrLane(0));
  XodrLanePtr lane_plus11(new XodrLane(1));
  XodrLanePtr lane_minus11(new XodrLane(-1));

  XodrLaneId l01 = r.add_lane(rid1, lane_01);
  XodrLaneId l11 = r.add_lane(rid1, lane_plus11);
  XodrLaneId lm11 = r.add_lane(rid1, lane_minus11);

  r.add_inner_neighbor(lm11, l01);
  r.add_outer_neighbor(lm11, l01);
  r.add_inner_neighbor(l01, l11);
  r.add_outer_neighbor(l01, l11);

  r.add_lane_successor(l10, l11);
  r.add_lane_successor(lm11, lm10);
  r.add_road_successor(l00, l01);

  int num_vertices = static_cast<int>(boost::num_vertices(r.get_lane_graph()));
  int num_edges = static_cast<int>(boost::num_edges(r.get_lane_graph()));

  r.print_graph("road_successors.dot");

  ASSERT_EQ(6, num_vertices);
  ASSERT_EQ(11, num_edges);

}

TEST(roadgraph, road_adjacency_simple)
{
  using namespace modules::world::map;
  Roadgraph r;

  XodrRoadId rid0 = 0;
  XodrLanePtr lane_00(new XodrLane(0));
  XodrLaneId l00 = r.add_lane(rid0, lane_00);

  XodrRoadId rid1 = 1;
  XodrLanePtr lane_01(new XodrLane(0));
  XodrLaneId l01 = r.add_lane(rid1, lane_01);

  r.add_lane_successor(l00, l01);
  r.add_road_successor(l00, l01);

  int num_vertices = static_cast<int>(boost::num_vertices(r.get_lane_graph()));
  int num_edges = static_cast<int>(boost::num_edges(r.get_lane_graph()));

  r.print_graph("road_successors_simple.dot");

  ASSERT_EQ(2, num_vertices);
  ASSERT_EQ(2, num_edges);

}


TEST(roadgraph, get_successor_lane_test)
{
  using namespace modules::world::map;
  Roadgraph r;

  XodrRoadId rid0 = 0;
  XodrLanePtr lane_00(new XodrLane(0));
  XodrLanePtr lane_10(new XodrLane(1));
  XodrLaneId l00 = r.add_lane(rid0, lane_00);
  XodrLaneId l10 = r.add_lane(rid0, lane_10);

  XodrRoadId rid1 = 1;
  XodrLanePtr lane_01(new XodrLane(0));
  XodrLaneId l01 = r.add_lane(rid1, lane_01);

  r.add_lane_successor(l00, l01);
  r.add_inner_neighbor(l00, l10);
  r.add_road_successor(l00, l01);

  std::vector<XodrLaneId> suc = r.get_successor_lanes(l00);
  ASSERT_TRUE(suc.size() == 1);
  ASSERT_TRUE(suc[0] == l01);
}

TEST(roadgraph, get_predecessor_lane_test)
{
  using namespace modules::world::map;
  Roadgraph r;

  XodrRoadId rid0 = 0;
  XodrLanePtr lane_00(new XodrLane(0));
  XodrLanePtr lane_10(new XodrLane(1));
  XodrLaneId l00 = r.add_lane(rid0, lane_00);
  XodrLaneId l10 = r.add_lane(rid0, lane_10);

  XodrRoadId rid1 = 1;
  XodrLanePtr lane_01(new XodrLane(0));
  XodrLaneId l01 = r.add_lane(rid1, lane_01);

  r.add_lane_successor(l00, l01);
  r.add_inner_neighbor(l00, l10);
  r.add_road_successor(l00, l01);

  std::vector<XodrLaneId> pre = r.get_predecessor_lanes(l01);
  ASSERT_TRUE(pre.size() == 1);
  ASSERT_TRUE(pre[0] == l00);
}


TEST(roadgraph, get_inner_neighbor_test)
{
  using namespace modules::world::map;
  Roadgraph r;

  XodrRoadId rid0 = 0;
  XodrLanePtr lane_00(new XodrLane(0));
  XodrLanePtr lane_10(new XodrLane(1));
  XodrLaneId l00 = r.add_lane(rid0, lane_00);
  XodrLaneId l10 = r.add_lane(rid0, lane_10);

  XodrRoadId rid1 = 1;
  XodrLanePtr lane_01(new XodrLane(0));
  XodrLaneId l01 = r.add_lane(rid1, lane_01);

  r.add_lane_successor(l00, l01);
  r.add_inner_neighbor(l00, l10);
  r.add_road_successor(l00, l01);

  std::pair<XodrLaneId, bool> in = r.get_inner_neighbor(l10);
  ASSERT_TRUE(in.first == l00);
  ASSERT_TRUE(in.second);
}

TEST(roadgraph, get_outer_neighbor_test)
{
  using namespace modules::world::map;
  Roadgraph r;

  XodrRoadId rid0 = 0;
  XodrLanePtr lane_00(new XodrLane(0));
  XodrLanePtr lane_10(new XodrLane(1));
  XodrLaneId l00 = r.add_lane(rid0, lane_00);
  XodrLaneId l10 = r.add_lane(rid0, lane_10);

  r.add_inner_neighbor(l00, l10);
  r.add_outer_neighbor(l00, l10);

  r.print_graph("get_outer_neighbor_test.dot");

  std::pair<XodrLaneId, bool> in = r.get_outer_neighbor(l00);
  ASSERT_TRUE(in.first == l10);
  ASSERT_TRUE(in.second);

}

TEST(roadgraph, get_inner_neighbor_test_planview)
{
  using namespace modules::world::map;
  Roadgraph r;

  XodrRoadId rid0 = 0;
  XodrLanePtr lane_m1(new XodrLane(-1));
  XodrLanePtr lane_0(new XodrLane(0));
  XodrLanePtr lane_1(new XodrLane(1));
  XodrLaneId l0 = r.add_lane(rid0, lane_0);
  XodrLaneId l1 = r.add_lane(rid0, lane_1);
  XodrLaneId lm1 = r.add_lane(rid0, lane_m1);

  r.add_inner_neighbor(l0, l1);
  r.add_inner_neighbor(l0, lm1);
  r.add_outer_neighbor(l0, l1);
  r.add_outer_neighbor(l0, lm1);

  r.print_graph("get_inner_neighbor_test_planview.dot");

  std::pair<XodrLaneId, bool> in = r.get_inner_neighbor(l1);
  ASSERT_TRUE(in.first == l0);
  ASSERT_TRUE(in.second);

  std::pair<XodrLaneId, bool> in2 = r.get_inner_neighbor(lm1);
  ASSERT_TRUE(in2.first == l0);
  ASSERT_TRUE(in2.second);

}

TEST(roadgraph, get_lane_planview_test)
{
  using namespace modules::world::map;
  Roadgraph r;

  XodrRoadId rid0 = 0;
  XodrLanePtr lane_0(new XodrLane(0));
  XodrLanePtr lane_1(new XodrLane(1));
  XodrLanePtr lane_2(new XodrLane(2));

  XodrLaneId l0 = r.add_lane(rid0, lane_0);
  XodrLaneId l1 = r.add_lane(rid0, lane_1);
  XodrLaneId l2 = r.add_lane(rid0, lane_2);

  r.add_inner_neighbor(l0, l1);
  r.add_inner_neighbor(l1, l2);
  r.add_outer_neighbor(l0, l1);
  r.add_outer_neighbor(l1, l2);

  r.print_graph("get_lane_planview_test.dot");

  std::pair<XodrLaneId, bool> in = r.GetPlanViewForLaneId(l0);
  ASSERT_TRUE(in.first == l0);
  ASSERT_TRUE(in.second);

  std::pair<XodrLaneId, bool> in1 = r.GetPlanViewForLaneId(l1);
  ASSERT_TRUE(in1.first == l0);
  ASSERT_TRUE(in1.second);

  std::pair<XodrLaneId, bool> in2 = r.GetPlanViewForLaneId(l2);
  ASSERT_TRUE(in2.first == l0);
  ASSERT_TRUE(in2.second);
}

TEST(roadgraph, get_all_neighbors_test)
{
  using namespace modules::world::map;
  Roadgraph r;

  XodrRoadId rid0 = 0;
  XodrLanePtr lane_0(new XodrLane(0));
  XodrLanePtr lane_1(new XodrLane(1));
  XodrLanePtr lane_2(new XodrLane(2));
  XodrLanePtr lane_3(new XodrLane(3));
  XodrLaneId l0 = r.add_lane(rid0, lane_0);
  XodrLaneId l1 = r.add_lane(rid0, lane_1);
  XodrLaneId l2 = r.add_lane(rid0, lane_2);
  XodrLaneId l3 = r.add_lane(rid0, lane_3);

  r.add_inner_neighbor(l0, l1);
  r.add_inner_neighbor(l1, l2);
  r.add_inner_neighbor(l2, l3);
  r.add_outer_neighbor(l0, l1);
  r.add_outer_neighbor(l1, l2);
  r.add_outer_neighbor(l2, l3);

  std::vector<XodrLaneId> neighbors_1 = r.get_all_neighbors(l1);
  std::vector<XodrLaneId> neighbors_2 = r.get_all_neighbors(l2);
  std::vector<XodrLaneId> neighbors_3 = r.get_all_neighbors(l3);
  
  // Vector elements are the neighbors, and not the lane itself or the plan view
  ASSERT_EQ(2, neighbors_1.size());
  ASSERT_NE(neighbors_1.end(), find(neighbors_1.begin(), neighbors_1.end(), l2));
  ASSERT_NE(neighbors_1.end(), find(neighbors_1.begin(), neighbors_1.end(), l3));

  ASSERT_EQ(2, neighbors_2.size());
  ASSERT_NE(neighbors_2.end(), find(neighbors_2.begin(), neighbors_2.end(), l1));
  ASSERT_NE(neighbors_2.end(), find(neighbors_2.begin(), neighbors_2.end(), l3));

  ASSERT_EQ(2, neighbors_3.size());
  ASSERT_NE(neighbors_3.end(), find(neighbors_3.begin(), neighbors_3.end(), l1));
  ASSERT_NE(neighbors_3.end(), find(neighbors_3.begin(), neighbors_3.end(), l2));

  // Neighbors of planview -> expect failure
  ASSERT_THROW(r.get_all_neighbors(l0), std::runtime_error);
}

TEST(roadgraph, find_path_test)
{
  using namespace modules::world::map;
  Roadgraph r;

  XodrLanePtr lane_0(new XodrLane()); lane_0->set_id(10); lane_0->set_lane_type(XodrLaneType::DRIVING);
  XodrLanePtr lane_1(new XodrLane()); lane_1->set_id(20); lane_1->set_lane_type(XodrLaneType::DRIVING);
  XodrLanePtr lane_2(new XodrLane()); lane_2->set_id(30); lane_2->set_lane_type(XodrLaneType::DRIVING);
  XodrLanePtr lane_3(new XodrLane()); lane_3->set_id(40); lane_3->set_lane_type(XodrLaneType::DRIVING);
  XodrLanePtr lane_4(new XodrLane()); lane_4->set_id(50); lane_4->set_lane_type(XodrLaneType::DRIVING);
  XodrLanePtr lane_5(new XodrLane()); lane_5->set_id(60); lane_5->set_lane_type(XodrLaneType::DRIVING);
  XodrLaneId l0 = r.add_lane(0, lane_0);
  XodrLaneId l1 = r.add_lane(1, lane_1);
  XodrLaneId l2 = r.add_lane(2, lane_2);
  XodrLaneId l3 = r.add_lane(3, lane_3);
  XodrLaneId l4 = r.add_lane(4, lane_4);
  XodrLaneId l5 = r.add_lane(5, lane_5);

  r.add_lane_successor(l0, l1);
  r.add_lane_successor(l1, l2);
  r.add_lane_successor(l2, l3);
  r.add_lane_successor(l3, l4);
  r.add_lane_successor(l0, l5);
  r.add_lane_successor(l5, l3);


  std::vector<XodrLaneId> path = r.find_path<TypeDrivingAndEdgeTypeLaneSuccessor>(l0,l4);
  ASSERT_EQ(4, path.size());
  ASSERT_EQ(l0, path[0]);
  ASSERT_EQ(path[1], l5);
  ASSERT_EQ(path[2], l3);
  ASSERT_EQ(path[3], l4);

  path.clear();
  path = r.find_path<TypeDrivingAndEdgeTypeLaneSuccessor>(l0,l2);
  ASSERT_EQ(3, path.size());
  ASSERT_EQ(path[0], l0);
  ASSERT_EQ(path[1], l1);
  ASSERT_EQ(path[2], l2);
  
}

TEST(roadgraph, FindRoadPath_test)
{
  using namespace modules::world::map;
  Roadgraph r;

  XodrRoadId rid0 = 0;
  XodrLanePtr lane_00(new XodrLane(0));
  XodrLanePtr lane_plus10(new XodrLane(1));

  XodrLaneId l00 = r.add_lane(rid0, lane_00);
  XodrLaneId l10 = r.add_lane(rid0, lane_plus10);

  r.add_inner_neighbor(l00, l10);
  r.add_outer_neighbor(l00, l10);

  XodrRoadId rid1 = 1;
  XodrLanePtr lane_01(new XodrLane(0));
  XodrLanePtr lane_plus11(new XodrLane(1));

  XodrLaneId l01 = r.add_lane(rid1, lane_01);
  XodrLaneId l11 = r.add_lane(rid1, lane_plus11);

  r.add_inner_neighbor(l01, l11);
  r.add_outer_neighbor(l01, l11);

  XodrRoadId rid2 = 2;
  XodrLanePtr lane_02(new XodrLane(0));
  XodrLanePtr lane_plus12(new XodrLane(1));

  XodrLaneId l02 = r.add_lane(rid2, lane_02);
  XodrLaneId l12 = r.add_lane(rid2, lane_plus12);

  r.add_inner_neighbor(l02, l12);
  r.add_outer_neighbor(l02, l12);

  r.add_lane_successor(l10, l11);
  r.add_lane_successor(l11, l12);
  r.add_road_successor(l00, l01);
  r.add_road_successor(l01, l02);
 
  r.print_graph("FindRoadPath_test.dot");

  std::vector<XodrLaneId> path = r.find_path<EdgeTypeRoadSuccessor>(l10,l12);
  ASSERT_EQ(0, path.size());

  path.clear();
  path = r.find_path<EdgeTypeRoadSuccessor>(l00,l01);
  ASSERT_EQ(2, path.size());
  ASSERT_EQ(path[0], l00);
  ASSERT_EQ(path[1], l01);

  path.clear();
  path = r.find_path<EdgeTypeRoadSuccessor>(l00,l02);
  ASSERT_EQ(3, path.size());
  ASSERT_EQ(path[0], l00);
  ASSERT_EQ(path[1], l01);
  ASSERT_EQ(path[2], l02);

  std::vector<XodrRoadId> path_r = r.FindRoadPath(rid0, rid1);
  ASSERT_EQ(2, path_r.size());
  ASSERT_EQ(path_r[0], rid0);
  ASSERT_EQ(path_r[1], rid1);
  
  path_r.clear();
  path_r = r.FindRoadPath(rid0, rid2);
  ASSERT_EQ(3, path_r.size());
  ASSERT_EQ(path_r[0], rid0);
  ASSERT_EQ(path_r[1], rid1);
  ASSERT_EQ(path_r[2], rid2);

  auto nextlane = r.GetNextLane(path_r, l10);
  ASSERT_TRUE(nextlane.second);
  ASSERT_EQ(nextlane.first, l11);

  nextlane = r.GetNextLane(path_r, l11);
  ASSERT_TRUE(nextlane.second);
  ASSERT_EQ(nextlane.first, l12);

  nextlane = r.GetNextLane(path_r, l12);
  ASSERT_FALSE(nextlane.second);
  
}

TEST(roadgraph, find_path_in_unconnected_graph_test)
{
  using namespace modules::world::map;
  Roadgraph r;

  XodrLanePtr lane_0(new XodrLane()); lane_0->set_id(10); lane_0->set_lane_type(XodrLaneType::DRIVING);
  XodrLanePtr lane_1(new XodrLane()); lane_1->set_id(20); lane_1->set_lane_type(XodrLaneType::DRIVING);
  XodrLanePtr lane_2(new XodrLane()); lane_2->set_id(30); lane_2->set_lane_type(XodrLaneType::DRIVING);
  XodrLanePtr lane_3(new XodrLane()); lane_3->set_id(40); lane_3->set_lane_type(XodrLaneType::DRIVING);
  XodrLanePtr lane_4(new XodrLane()); lane_4->set_id(50); lane_4->set_lane_type(XodrLaneType::DRIVING);
  XodrLaneId l0 = r.add_lane(0, lane_0);
  XodrLaneId l1 = r.add_lane(1, lane_1);
  XodrLaneId l2 = r.add_lane(2, lane_2);
  XodrLaneId l3 = r.add_lane(3, lane_3);
  XodrLaneId l4 = r.add_lane(4, lane_4);

  r.add_lane_successor(l0, l1);
  r.add_lane_successor(l1, l2);
  // r.add_lane_successor(l2, l3); REMOVING CONNECTION, thus not a valid path available
  r.add_lane_successor(l3, l4);

  std::vector<XodrLaneId> path = r.find_path<TypeDrivingAndEdgeTypeLaneSuccessor>(l0,l4);
  ASSERT_EQ(0, path.size());
 
}


TEST(roadgraph, find_drivable_path_test)
{
  using namespace modules::world::map;
  Roadgraph r;

  XodrLanePtr lane_0(new XodrLane()); lane_0->set_id(10); lane_0->set_lane_type(XodrLaneType::DRIVING);
  XodrLanePtr lane_1(new XodrLane()); lane_1->set_id(20); lane_1->set_lane_type(XodrLaneType::DRIVING);
  XodrLanePtr lane_2(new XodrLane()); lane_2->set_id(30); lane_2->set_lane_type(XodrLaneType::DRIVING);
  XodrLanePtr lane_3(new XodrLane()); lane_3->set_id(40); lane_3->set_lane_type(XodrLaneType::DRIVING);
  XodrLanePtr lane_4(new XodrLane()); lane_4->set_id(50); lane_4->set_lane_type(XodrLaneType::DRIVING);
  XodrLanePtr lane_5(new XodrLane()); lane_5->set_id(60); lane_5->set_lane_type(XodrLaneType::SIDEWALK);
  XodrLaneId l0 = r.add_lane(0, lane_0); 
  XodrLaneId l1 = r.add_lane(1, lane_1);
  XodrLaneId l2 = r.add_lane(2, lane_2);
  XodrLaneId l3 = r.add_lane(3, lane_3);
  XodrLaneId l4 = r.add_lane(4, lane_4);
  XodrLaneId l5 = r.add_lane(5, lane_5);

  r.add_lane_successor(l0, l1);
  r.add_lane_successor(l1, l2);
  r.add_lane_successor(l2, l3);
  r.add_lane_successor(l3, l4);
  r.add_lane_successor(l0, l5);
  r.add_lane_successor(l5, l3);

  std::vector<XodrLaneId> path = r.find_path<TypeDrivingAndEdgeTypeLaneSuccessor>(l0,l4);
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

  XodrLanePtr lane_0(new XodrLane()); lane_0->set_id(10); lane_0->set_lane_type(XodrLaneType::DRIVING);
  XodrLanePtr lane_1(new XodrLane()); lane_1->set_id(20); lane_1->set_lane_type(XodrLaneType::DRIVING);
  XodrLanePtr lane_2(new XodrLane()); lane_2->set_id(30); lane_2->set_lane_type(XodrLaneType::SIDEWALK);
  XodrLanePtr lane_3(new XodrLane()); lane_3->set_id(40); lane_3->set_lane_type(XodrLaneType::DRIVING);
  XodrLaneId l0 = r.add_lane(0, lane_0); 
  XodrLaneId l1 = r.add_lane(1, lane_1);
  XodrLaneId l2 = r.add_lane(2, lane_2);
  XodrLaneId l3 = r.add_lane(3, lane_3);

  r.add_lane_successor(l0, l1);
  r.add_lane_successor(l1, l2);
  r.add_lane_successor(l2, l3);

  std::vector<XodrLaneId> path = r.find_path<TypeDrivingAndEdgeTypeLaneSuccessor>(l0,l3);
  ASSERT_EQ(0, path.size());
}

TEST(roadgraph, find_drivable_path_invalid_final_vertex_test)
{
  using namespace modules::world::map;
  Roadgraph r;

  XodrLanePtr lane_0(new XodrLane()); lane_0->set_id(10); lane_0->set_lane_type(XodrLaneType::DRIVING);
  XodrLanePtr lane_1(new XodrLane()); lane_1->set_id(20); lane_1->set_lane_type(XodrLaneType::DRIVING);
  XodrLanePtr lane_2(new XodrLane()); lane_2->set_id(30); lane_2->set_lane_type(XodrLaneType::DRIVING);
  XodrLanePtr lane_3(new XodrLane()); lane_3->set_id(40); lane_3->set_lane_type(XodrLaneType::SIDEWALK);
  XodrLaneId l0 = r.add_lane(0, lane_0); 
  XodrLaneId l1 = r.add_lane(1, lane_1);
  XodrLaneId l2 = r.add_lane(2, lane_2);
  XodrLaneId l3 = r.add_lane(3, lane_3);

  r.add_lane_successor(l0, l1);
  r.add_lane_successor(l1, l2);
  r.add_lane_successor(l2, l3);

  std::vector<XodrLaneId> path = r.find_path<TypeDrivingAndEdgeTypeLaneSuccessor>(l0,l3);
  ASSERT_EQ(0, path.size());
}

TEST(roadgraph, find_path_along_neighbor_edges_test)
{
  using namespace modules::world::map;
  Roadgraph r;

  XodrLanePtr lane_0(new XodrLane()); lane_0->set_id(10); lane_0->set_lane_type(XodrLaneType::DRIVING);
  XodrLanePtr lane_1(new XodrLane()); lane_1->set_id(20); lane_1->set_lane_type(XodrLaneType::DRIVING);
  XodrLanePtr lane_2(new XodrLane()); lane_2->set_id(30); lane_2->set_lane_type(XodrLaneType::DRIVING);
  XodrLaneId l0 = r.add_lane(0, lane_0);
  XodrLaneId l1 = r.add_lane(1, lane_1);
  XodrLaneId l2 = r.add_lane(2, lane_2);

  r.add_lane_successor(l0, l1);
  r.add_inner_neighbor(l1, l2);
  r.add_outer_neighbor(l1, l2);

  // XodrLane l2 is not reachable using only successor edges, thus no valid path available
  std::vector<XodrLaneId> path = r.find_path<TypeDrivingAndEdgeTypeLaneSuccessor>(l0, l2);

  ASSERT_EQ(0, path.size());
}

TEST(roadgraph, get_driving_corridor_test)
{
  using namespace modules::world::map;
  Roadgraph r;

  XodrRoadId rid0 = 0;
  XodrLanePtr lane_00(new XodrLane()); 
  XodrLanePtr lane_10(new XodrLane());
  XodrLaneId l00 = r.add_lane(rid0, lane_00);
  XodrLaneId l10 = r.add_lane(rid0, lane_10);

  XodrRoadId rid1 = 1;
  XodrLanePtr lane_01(new XodrLane());
  XodrLanePtr lane_11(new XodrLane());
  XodrLaneId l01 = r.add_lane(rid1, lane_01);
  XodrLaneId l11 = r.add_lane(rid1, lane_11);

  r.add_lane_successor(l10, l11);
  r.add_inner_neighbor(l00, l10);
  r.add_inner_neighbor(l01, l11);

  // TODO(@fortiss): add new assertions as this has moved to MapInterface
  /*
  std::pair< std::vector<XodrLanePtr>, std::vector<XodrLanePtr> > horizon = r.ComputeXodrLaneBoundariesHorizon(l10, l11);

  ASSERT_TRUE(horizon.first.size()==2);
  ASSERT_TRUE(horizon.second.size()==2);
  */
}

TEST(roadgraph, generate_roadgraph_neighbours_test)
{
  using namespace modules::geometry;
  using namespace modules::world::opendrive;
  using namespace modules::world::map;

  OpenDriveMapPtr open_drive_map(new OpenDriveMap());
    //! ROAD 1
  PlanViewPtr p(new PlanView());
  p->add_line(Point2d(0.0f, 0.0f), 0.0f, 10.0f);

  //! XodrLane-Section 1
  XodrLaneSectionPtr ls(new XodrLaneSection(0.0));

  //! XodrLane
  XodrLanePtr lane0(new XodrLane(0));
  lane0->set_line(p->get_reference_line());

  XodrLaneOffset off = {1.0f, 0.0f, 0.0f, 0.0f};
  XodrLaneWidth lane_width_1 = {0, 10, off};
  
  XodrLanePtr lane1 = create_lane_from_lane_width(-1, p->get_reference_line(), lane_width_1, 0.05);
  XodrLanePtr lane2 = create_lane_from_lane_width(1, p->get_reference_line(), lane_width_1, 0.05);
  XodrLanePtr lane3 = create_lane_from_lane_width(2, lane2->get_line(), lane_width_1, 0.05);

  ls->add_lane(lane1);
  ls->add_lane(lane2);
  ls->add_lane(lane3);

  XodrRoadPtr r(new XodrRoad("highway", 100));
  r->set_plan_view(p);
  r->add_lane_section(ls);

  open_drive_map->add_road(r);

  Roadgraph rg;
  rg.Generate(open_drive_map);

  std::pair<XodrLaneId, bool> in1 = rg.get_inner_neighbor(lane1->get_id());
  // There is no inner neighbor for lane1, as lane1 and lane2 are left and right of the planview
  ASSERT_FALSE(in1.second);

  std::pair<XodrLaneId, bool> out2 = rg.get_outer_neighbor(lane2->get_id());
  ASSERT_TRUE(out2.second);
  ASSERT_TRUE(out2.first == lane3->get_id());

}


TEST(roadgraph, generate_roadgraph_successors_test)
{
  using namespace modules::geometry;
  using namespace modules::world::opendrive;
  using namespace modules::world::map;

  OpenDriveMapPtr open_drive_map(new OpenDriveMap());
  //! ROAD 1
  PlanViewPtr p(new PlanView());
  p->add_line(Point2d(0.0f, 0.0f), 0.0f, 10.0f);

  //! XodrLane-Section 1
  XodrLaneSectionPtr ls(new XodrLaneSection(0.0));
  
  //! XodrLane
  XodrLanePtr lane00(new XodrLane(0));
  lane00->set_line(p->get_reference_line());

  XodrLaneOffset off = {1.0f, 0.0f, 0.0f, 0.0f};
  XodrLaneWidth lane_width_1 = {0, 10, off};
  XodrLanePtr lane10 = create_lane_from_lane_width(1, p->get_reference_line(), lane_width_1, 0.05);
  XodrLaneLink ll1;
  ll1.from_position = 1;
  ll1.to_position = 1;
  lane10->set_link(ll1);

  ls->add_lane(lane00);
  ls->add_lane(lane10);

  XodrRoadPtr r(new XodrRoad("highway", 100));
  r->set_plan_view(p);
  r->add_lane_section(ls);

  //! ROAD 2
  PlanViewPtr p2(new PlanView());
  p2->add_line(Point2d(10.0f, 0.0f), 0.0f, 20.0f);

  //! XodrLane-Section 2
  XodrLaneSectionPtr ls2(new XodrLaneSection(0.0));
  
  //! XodrLane
  XodrLanePtr lane01(new XodrLane(0));
  lane01->set_line(p2->get_reference_line());

  XodrLaneOffset off2 = {1.0f, 0.0f, 0.0f, 0.0f};
  XodrLaneWidth lane_width_2 = {0, 10, off};
  XodrLanePtr lane11 = create_lane_from_lane_width(1, p2->get_reference_line(), lane_width_2, 0.05);
  XodrLaneLink ll2;
  ll2.from_position = 1;
  ll2.to_position = 1;
  lane11->set_link(ll2);
  ls2->add_lane(lane01);
  ls2->add_lane(lane11);

  XodrRoadPtr r2(new XodrRoad("highway", 101));
  r2->set_plan_view(p2);
  r2->add_lane_section(ls2);
  
  XodrRoadLinkInfo rli_succ(r2->get_id(), "road");
  XodrRoadLink rl;
  rl.set_successor(rli_succ);
  r->set_link(rl);

  open_drive_map->add_road(r);
  open_drive_map->add_road(r2);

  Roadgraph rg;
  rg.Generate(open_drive_map);
 
  rg.print_graph("generate_roadgraph_successors_test.dot");

  std::vector<XodrLaneId> suc = rg.get_successor_lanes(lane10->get_id());
  std::cout << "suc: " << suc.size() << std::endl;
  ASSERT_TRUE(suc.size() == 1);
  ASSERT_TRUE(suc[0] == lane11->get_id());
  
  auto all_ids = rg.get_all_laneids();
  ASSERT_TRUE(all_ids.size() == 2);
}
