// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/world/map/roadgraph.hpp"
#include <utility>
#include "bark/world/opendrive/opendrive.hpp"
#include "bark/world/tests/make_test_xodr_map.hpp"
#include "gtest/gtest.h"

TEST(roadgraph, road_creation) {
  using namespace bark::world::map;
  Roadgraph r;

  XodrRoadId rid = 0;
  XodrLanePtr lane_0(new XodrLane());
  XodrLanePtr lane_plus1(new XodrLane());
  XodrLanePtr lane_plus2(new XodrLane());
  XodrLanePtr lane_minus1(new XodrLane());

  XodrLaneId l0 = r.AddLane(rid, lane_0);
  XodrLaneId l1 = r.AddLane(rid, lane_plus1);
  XodrLaneId l2 = r.AddLane(rid, lane_plus2);
  XodrLaneId lm1 = r.AddLane(rid, lane_minus1);

  r.AddInnerNeighbor(lm1, l0);
  r.AddOuterNeighbor(lm1, l0);
  r.AddInnerNeighbor(l0, l1);
  r.AddOuterNeighbor(l0, l1);
  r.AddInnerNeighbor(l1, l2);
  r.AddOuterNeighbor(l1, l2);

  int num_vertices = static_cast<int>(boost::num_vertices(r.GetLaneGraph()));
  int num_edges = static_cast<int>(boost::num_edges(r.GetLaneGraph()));

  r.PrintGraph("road_creation.dot");

  ASSERT_EQ(4, num_vertices);
  ASSERT_EQ(6, num_edges);
}

TEST(roadgraph, road_adjacency) {
  using namespace bark::world::map;
  Roadgraph r;

  XodrRoadId rid0 = 0;
  XodrLanePtr lane_00(new XodrLane(0));
  XodrLanePtr lane_plus10(new XodrLane(1));
  XodrLanePtr lane_minus10(new XodrLane(-1));

  XodrLaneId l00 = r.AddLane(rid0, lane_00);
  XodrLaneId l10 = r.AddLane(rid0, lane_plus10);
  XodrLaneId lm10 = r.AddLane(rid0, lane_minus10);

  r.AddInnerNeighbor(lm10, l00);
  r.AddOuterNeighbor(lm10, l00);
  r.AddInnerNeighbor(l00, l10);
  r.AddOuterNeighbor(l00, l10);

  XodrRoadId rid1 = 1;
  XodrLanePtr lane_01(new XodrLane(0));
  XodrLanePtr lane_plus11(new XodrLane(1));
  XodrLanePtr lane_minus11(new XodrLane(-1));

  XodrLaneId l01 = r.AddLane(rid1, lane_01);
  XodrLaneId l11 = r.AddLane(rid1, lane_plus11);
  XodrLaneId lm11 = r.AddLane(rid1, lane_minus11);

  r.AddInnerNeighbor(lm11, l01);
  r.AddOuterNeighbor(lm11, l01);
  r.AddInnerNeighbor(l01, l11);
  r.AddOuterNeighbor(l01, l11);

  r.AddLaneSuccessor(l10, l11);
  r.AddLaneSuccessor(lm11, lm10);
  r.AddRoadSuccessor(l00, l01);

  int num_vertices = static_cast<int>(boost::num_vertices(r.GetLaneGraph()));
  int num_edges = static_cast<int>(boost::num_edges(r.GetLaneGraph()));

  r.PrintGraph("road_successors.dot");

  ASSERT_EQ(6, num_vertices);
  ASSERT_EQ(11, num_edges);
}

TEST(roadgraph, road_adjacency_simple) {
  using namespace bark::world::map;
  Roadgraph r;

  XodrRoadId rid0 = 0;
  XodrLanePtr lane_00(new XodrLane(0));
  XodrLaneId l00 = r.AddLane(rid0, lane_00);

  XodrRoadId rid1 = 1;
  XodrLanePtr lane_01(new XodrLane(0));
  XodrLaneId l01 = r.AddLane(rid1, lane_01);

  r.AddLaneSuccessor(l00, l01);
  r.AddRoadSuccessor(l00, l01);

  int num_vertices = static_cast<int>(boost::num_vertices(r.GetLaneGraph()));
  int num_edges = static_cast<int>(boost::num_edges(r.GetLaneGraph()));

  r.PrintGraph("road_successors_simple.dot");

  ASSERT_EQ(2, num_vertices);
  ASSERT_EQ(2, num_edges);
}

TEST(roadgraph, GetSuccessor_lane_test) {
  using namespace bark::world::map;
  Roadgraph r;

  XodrRoadId rid0 = 0;
  XodrLanePtr lane_00(new XodrLane(0));
  XodrLanePtr lane_10(new XodrLane(1));
  XodrLaneId l00 = r.AddLane(rid0, lane_00);
  XodrLaneId l10 = r.AddLane(rid0, lane_10);

  XodrRoadId rid1 = 1;
  XodrLanePtr lane_01(new XodrLane(0));
  XodrLaneId l01 = r.AddLane(rid1, lane_01);

  r.AddLaneSuccessor(l00, l01);
  r.AddInnerNeighbor(l00, l10);
  r.AddRoadSuccessor(l00, l01);

  std::vector<XodrLaneId> suc = r.GetSuccessorLanes(l00);
  ASSERT_TRUE(suc.size() == 1);
  ASSERT_TRUE(suc[0] == l01);
}

TEST(roadgraph, GetPredecessor_lane_test) {
  using namespace bark::world::map;
  Roadgraph r;

  XodrRoadId rid0 = 0;
  XodrLanePtr lane_00(new XodrLane(0));
  XodrLanePtr lane_10(new XodrLane(1));
  XodrLaneId l00 = r.AddLane(rid0, lane_00);
  XodrLaneId l10 = r.AddLane(rid0, lane_10);

  XodrRoadId rid1 = 1;
  XodrLanePtr lane_01(new XodrLane(0));
  XodrLaneId l01 = r.AddLane(rid1, lane_01);

  r.AddLaneSuccessor(l00, l01);
  r.AddInnerNeighbor(l00, l10);
  r.AddRoadSuccessor(l00, l01);

  std::vector<XodrLaneId> pre = r.GetPredecessorLanes(l01);
  ASSERT_TRUE(pre.size() == 1);
  ASSERT_TRUE(pre[0] == l00);
}

TEST(roadgraph, GetInnerNeighbor_test) {
  using namespace bark::world::map;
  Roadgraph r;

  XodrRoadId rid0 = 0;
  XodrLanePtr lane_00(new XodrLane(0));
  XodrLanePtr lane_10(new XodrLane(1));
  XodrLaneId l00 = r.AddLane(rid0, lane_00);
  XodrLaneId l10 = r.AddLane(rid0, lane_10);

  XodrRoadId rid1 = 1;
  XodrLanePtr lane_01(new XodrLane(0));
  XodrLaneId l01 = r.AddLane(rid1, lane_01);

  r.AddLaneSuccessor(l00, l01);
  r.AddInnerNeighbor(l00, l10);
  r.AddRoadSuccessor(l00, l01);

  std::pair<XodrLaneId, bool> in = r.GetInnerNeighbor(l10);
  ASSERT_TRUE(in.first == l00);
  ASSERT_TRUE(in.second);
}

TEST(roadgraph, GetOuterNeighbor_test) {
  using namespace bark::world::map;
  Roadgraph r;

  XodrRoadId rid0 = 0;
  XodrLanePtr lane_00(new XodrLane(0));
  XodrLanePtr lane_10(new XodrLane(1));
  XodrLaneId l00 = r.AddLane(rid0, lane_00);
  XodrLaneId l10 = r.AddLane(rid0, lane_10);

  r.AddInnerNeighbor(l00, l10);
  r.AddOuterNeighbor(l00, l10);

  r.PrintGraph("GetOuterNeighbor_test.dot");

  std::pair<XodrLaneId, bool> in = r.GetOuterNeighbor(l00);
  ASSERT_TRUE(in.first == l10);
  ASSERT_TRUE(in.second);
}

TEST(roadgraph, GetInnerNeighbor_test_planview) {
  using namespace bark::world::map;
  Roadgraph r;

  XodrRoadId rid0 = 0;
  XodrLanePtr lane_m1(new XodrLane(-1));
  XodrLanePtr lane_0(new XodrLane(0));
  XodrLanePtr lane_1(new XodrLane(1));
  XodrLaneId l0 = r.AddLane(rid0, lane_0);
  XodrLaneId l1 = r.AddLane(rid0, lane_1);
  XodrLaneId lm1 = r.AddLane(rid0, lane_m1);

  r.AddInnerNeighbor(l0, l1);
  r.AddInnerNeighbor(l0, lm1);
  r.AddOuterNeighbor(l0, l1);
  r.AddOuterNeighbor(l0, lm1);

  r.PrintGraph("GetInnerNeighbor_test_planview.dot");

  std::pair<XodrLaneId, bool> in = r.GetInnerNeighbor(l1);
  ASSERT_TRUE(in.first == l0);
  ASSERT_TRUE(in.second);

  std::pair<XodrLaneId, bool> in2 = r.GetInnerNeighbor(lm1);
  ASSERT_TRUE(in2.first == l0);
  ASSERT_TRUE(in2.second);
}

TEST(roadgraph, GetLane_planview_test) {
  using namespace bark::world::map;
  Roadgraph r;

  XodrRoadId rid0 = 0;
  XodrLanePtr lane_0(new XodrLane(0));
  XodrLanePtr lane_1(new XodrLane(1));
  XodrLanePtr lane_2(new XodrLane(2));

  XodrLaneId l0 = r.AddLane(rid0, lane_0);
  XodrLaneId l1 = r.AddLane(rid0, lane_1);
  XodrLaneId l2 = r.AddLane(rid0, lane_2);

  r.AddInnerNeighbor(l0, l1);
  r.AddInnerNeighbor(l1, l2);
  r.AddOuterNeighbor(l0, l1);
  r.AddOuterNeighbor(l1, l2);

  r.PrintGraph("GetLane_planview_test.dot");

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

TEST(roadgraph, GetAllNeighbors_test) {
  using namespace bark::world::map;
  Roadgraph r;

  XodrRoadId rid0 = 0;
  XodrLanePtr lane_0(new XodrLane(0));
  XodrLanePtr lane_1(new XodrLane(1));
  XodrLanePtr lane_2(new XodrLane(2));
  XodrLanePtr lane_3(new XodrLane(3));
  XodrLaneId l0 = r.AddLane(rid0, lane_0);
  XodrLaneId l1 = r.AddLane(rid0, lane_1);
  XodrLaneId l2 = r.AddLane(rid0, lane_2);
  XodrLaneId l3 = r.AddLane(rid0, lane_3);

  r.AddInnerNeighbor(l0, l1);
  r.AddInnerNeighbor(l1, l2);
  r.AddInnerNeighbor(l2, l3);
  r.AddOuterNeighbor(l0, l1);
  r.AddOuterNeighbor(l1, l2);
  r.AddOuterNeighbor(l2, l3);

  std::vector<XodrLaneId> neighbors_1 = r.GetAllNeighbors(l1);
  std::vector<XodrLaneId> neighbors_2 = r.GetAllNeighbors(l2);
  std::vector<XodrLaneId> neighbors_3 = r.GetAllNeighbors(l3);

  // Vector elements are the neighbors, and not the lane itself or the plan view
  ASSERT_EQ(2, neighbors_1.size());
  ASSERT_NE(neighbors_1.end(),
            find(neighbors_1.begin(), neighbors_1.end(), l2));
  ASSERT_NE(neighbors_1.end(),
            find(neighbors_1.begin(), neighbors_1.end(), l3));

  ASSERT_EQ(2, neighbors_2.size());
  ASSERT_NE(neighbors_2.end(),
            find(neighbors_2.begin(), neighbors_2.end(), l1));
  ASSERT_NE(neighbors_2.end(),
            find(neighbors_2.begin(), neighbors_2.end(), l3));

  ASSERT_EQ(2, neighbors_3.size());
  ASSERT_NE(neighbors_3.end(),
            find(neighbors_3.begin(), neighbors_3.end(), l1));
  ASSERT_NE(neighbors_3.end(),
            find(neighbors_3.begin(), neighbors_3.end(), l2));

  // Neighbors of planview -> expect failure
  ASSERT_THROW(r.GetAllNeighbors(l0), std::runtime_error);
}

TEST(roadgraph, FindPath_test) {
  using namespace bark::world::map;
  Roadgraph r;

  XodrLanePtr lane_0(new XodrLane());
  lane_0->SetId(10);
  lane_0->SetLaneType(XodrLaneType::DRIVING);
  XodrLanePtr lane_1(new XodrLane());
  lane_1->SetId(20);
  lane_1->SetLaneType(XodrLaneType::DRIVING);
  XodrLanePtr lane_2(new XodrLane());
  lane_2->SetId(30);
  lane_2->SetLaneType(XodrLaneType::DRIVING);
  XodrLanePtr lane_3(new XodrLane());
  lane_3->SetId(40);
  lane_3->SetLaneType(XodrLaneType::DRIVING);
  XodrLanePtr lane_4(new XodrLane());
  lane_4->SetId(50);
  lane_4->SetLaneType(XodrLaneType::DRIVING);
  XodrLanePtr lane_5(new XodrLane());
  lane_5->SetId(60);
  lane_5->SetLaneType(XodrLaneType::DRIVING);
  XodrLaneId l0 = r.AddLane(0, lane_0);
  XodrLaneId l1 = r.AddLane(1, lane_1);
  XodrLaneId l2 = r.AddLane(2, lane_2);
  XodrLaneId l3 = r.AddLane(3, lane_3);
  XodrLaneId l4 = r.AddLane(4, lane_4);
  XodrLaneId l5 = r.AddLane(5, lane_5);

  r.AddLaneSuccessor(l0, l1);
  r.AddLaneSuccessor(l1, l2);
  r.AddLaneSuccessor(l2, l3);
  r.AddLaneSuccessor(l3, l4);
  r.AddLaneSuccessor(l0, l5);
  r.AddLaneSuccessor(l5, l3);

  std::vector<XodrLaneId> path =
      r.FindPath<TypeDrivingAndEdgeTypeLaneSuccessor>(l0, l4);
  ASSERT_EQ(4, path.size());
  ASSERT_EQ(l0, path[0]);
  ASSERT_EQ(path[1], l5);
  ASSERT_EQ(path[2], l3);
  ASSERT_EQ(path[3], l4);

  path.clear();
  path = r.FindPath<TypeDrivingAndEdgeTypeLaneSuccessor>(l0, l2);
  ASSERT_EQ(3, path.size());
  ASSERT_EQ(path[0], l0);
  ASSERT_EQ(path[1], l1);
  ASSERT_EQ(path[2], l2);
}

TEST(roadgraph, FindRoadPath_test) {
  using namespace bark::world::map;
  Roadgraph r;

  XodrRoadId rid0 = 0;
  XodrLanePtr lane_00(new XodrLane(0));
  XodrLanePtr lane_plus10(new XodrLane(1));

  XodrLaneId l00 = r.AddLane(rid0, lane_00);
  XodrLaneId l10 = r.AddLane(rid0, lane_plus10);

  r.AddInnerNeighbor(l00, l10);
  r.AddOuterNeighbor(l00, l10);

  XodrRoadId rid1 = 1;
  XodrLanePtr lane_01(new XodrLane(0));
  XodrLanePtr lane_plus11(new XodrLane(1));

  XodrLaneId l01 = r.AddLane(rid1, lane_01);
  XodrLaneId l11 = r.AddLane(rid1, lane_plus11);

  r.AddInnerNeighbor(l01, l11);
  r.AddOuterNeighbor(l01, l11);

  XodrRoadId rid2 = 2;
  XodrLanePtr lane_02(new XodrLane(0));
  XodrLanePtr lane_plus12(new XodrLane(1));

  XodrLaneId l02 = r.AddLane(rid2, lane_02);
  XodrLaneId l12 = r.AddLane(rid2, lane_plus12);

  r.AddInnerNeighbor(l02, l12);
  r.AddOuterNeighbor(l02, l12);

  r.AddLaneSuccessor(l10, l11);
  r.AddLaneSuccessor(l11, l12);
  r.AddRoadSuccessor(l00, l01);
  r.AddRoadSuccessor(l01, l02);

  r.PrintGraph("FindRoadPath_test.dot");

  std::vector<XodrLaneId> path = r.FindPath<EdgeTypeRoadSuccessor>(l10, l12);
  ASSERT_EQ(0, path.size());

  path.clear();
  path = r.FindPath<EdgeTypeRoadSuccessor>(l00, l01);
  ASSERT_EQ(2, path.size());
  ASSERT_EQ(path[0], l00);
  ASSERT_EQ(path[1], l01);

  path.clear();
  path = r.FindPath<EdgeTypeRoadSuccessor>(l00, l02);
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

TEST(roadgraph, FindPath_in_unconnected_graph_test) {
  using namespace bark::world::map;
  Roadgraph r;

  XodrLanePtr lane_0(new XodrLane());
  lane_0->SetId(10);
  lane_0->SetLaneType(XodrLaneType::DRIVING);
  XodrLanePtr lane_1(new XodrLane());
  lane_1->SetId(20);
  lane_1->SetLaneType(XodrLaneType::DRIVING);
  XodrLanePtr lane_2(new XodrLane());
  lane_2->SetId(30);
  lane_2->SetLaneType(XodrLaneType::DRIVING);
  XodrLanePtr lane_3(new XodrLane());
  lane_3->SetId(40);
  lane_3->SetLaneType(XodrLaneType::DRIVING);
  XodrLanePtr lane_4(new XodrLane());
  lane_4->SetId(50);
  lane_4->SetLaneType(XodrLaneType::DRIVING);
  XodrLaneId l0 = r.AddLane(0, lane_0);
  XodrLaneId l1 = r.AddLane(1, lane_1);
  XodrLaneId l2 = r.AddLane(2, lane_2);
  XodrLaneId l3 = r.AddLane(3, lane_3);
  XodrLaneId l4 = r.AddLane(4, lane_4);

  r.AddLaneSuccessor(l0, l1);
  r.AddLaneSuccessor(l1, l2);
  // r.AddLaneSuccessor(l2, l3); REMOVING CONNECTION, thus not a valid path
  // available
  r.AddLaneSuccessor(l3, l4);

  std::vector<XodrLaneId> path =
      r.FindPath<TypeDrivingAndEdgeTypeLaneSuccessor>(l0, l4);
  ASSERT_EQ(0, path.size());
}

TEST(roadgraph, find_drivable_path_test) {
  using namespace bark::world::map;
  Roadgraph r;

  XodrLanePtr lane_0(new XodrLane());
  lane_0->SetId(10);
  lane_0->SetLaneType(XodrLaneType::DRIVING);
  XodrLanePtr lane_1(new XodrLane());
  lane_1->SetId(20);
  lane_1->SetLaneType(XodrLaneType::DRIVING);
  XodrLanePtr lane_2(new XodrLane());
  lane_2->SetId(30);
  lane_2->SetLaneType(XodrLaneType::DRIVING);
  XodrLanePtr lane_3(new XodrLane());
  lane_3->SetId(40);
  lane_3->SetLaneType(XodrLaneType::DRIVING);
  XodrLanePtr lane_4(new XodrLane());
  lane_4->SetId(50);
  lane_4->SetLaneType(XodrLaneType::DRIVING);
  XodrLanePtr lane_5(new XodrLane());
  lane_5->SetId(60);
  lane_5->SetLaneType(XodrLaneType::SIDEWALK);
  XodrLaneId l0 = r.AddLane(0, lane_0);
  XodrLaneId l1 = r.AddLane(1, lane_1);
  XodrLaneId l2 = r.AddLane(2, lane_2);
  XodrLaneId l3 = r.AddLane(3, lane_3);
  XodrLaneId l4 = r.AddLane(4, lane_4);
  XodrLaneId l5 = r.AddLane(5, lane_5);

  r.AddLaneSuccessor(l0, l1);
  r.AddLaneSuccessor(l1, l2);
  r.AddLaneSuccessor(l2, l3);
  r.AddLaneSuccessor(l3, l4);
  r.AddLaneSuccessor(l0, l5);
  r.AddLaneSuccessor(l5, l3);

  std::vector<XodrLaneId> path =
      r.FindPath<TypeDrivingAndEdgeTypeLaneSuccessor>(l0, l4);
  ASSERT_EQ(5, path.size());
  ASSERT_EQ(path[0], l0);
  ASSERT_EQ(path[1], l1);
  ASSERT_EQ(path[2], l2);
  ASSERT_EQ(path[3], l3);
  ASSERT_EQ(path[4], l4);
}

TEST(roadgraph, find_drivable_path_in_unconnected_graph_test) {
  using namespace bark::world::map;
  Roadgraph r;

  XodrLanePtr lane_0(new XodrLane());
  lane_0->SetId(10);
  lane_0->SetLaneType(XodrLaneType::DRIVING);
  XodrLanePtr lane_1(new XodrLane());
  lane_1->SetId(20);
  lane_1->SetLaneType(XodrLaneType::DRIVING);
  XodrLanePtr lane_2(new XodrLane());
  lane_2->SetId(30);
  lane_2->SetLaneType(XodrLaneType::SIDEWALK);
  XodrLanePtr lane_3(new XodrLane());
  lane_3->SetId(40);
  lane_3->SetLaneType(XodrLaneType::DRIVING);
  XodrLaneId l0 = r.AddLane(0, lane_0);
  XodrLaneId l1 = r.AddLane(1, lane_1);
  XodrLaneId l2 = r.AddLane(2, lane_2);
  XodrLaneId l3 = r.AddLane(3, lane_3);

  r.AddLaneSuccessor(l0, l1);
  r.AddLaneSuccessor(l1, l2);
  r.AddLaneSuccessor(l2, l3);

  std::vector<XodrLaneId> path =
      r.FindPath<TypeDrivingAndEdgeTypeLaneSuccessor>(l0, l3);
  ASSERT_EQ(0, path.size());
}

TEST(roadgraph, find_drivable_path_invalid_final_vertex_test) {
  using namespace bark::world::map;
  Roadgraph r;

  XodrLanePtr lane_0(new XodrLane());
  lane_0->SetId(10);
  lane_0->SetLaneType(XodrLaneType::DRIVING);
  XodrLanePtr lane_1(new XodrLane());
  lane_1->SetId(20);
  lane_1->SetLaneType(XodrLaneType::DRIVING);
  XodrLanePtr lane_2(new XodrLane());
  lane_2->SetId(30);
  lane_2->SetLaneType(XodrLaneType::DRIVING);
  XodrLanePtr lane_3(new XodrLane());
  lane_3->SetId(40);
  lane_3->SetLaneType(XodrLaneType::SIDEWALK);
  XodrLaneId l0 = r.AddLane(0, lane_0);
  XodrLaneId l1 = r.AddLane(1, lane_1);
  XodrLaneId l2 = r.AddLane(2, lane_2);
  XodrLaneId l3 = r.AddLane(3, lane_3);

  r.AddLaneSuccessor(l0, l1);
  r.AddLaneSuccessor(l1, l2);
  r.AddLaneSuccessor(l2, l3);

  std::vector<XodrLaneId> path =
      r.FindPath<TypeDrivingAndEdgeTypeLaneSuccessor>(l0, l3);
  ASSERT_EQ(0, path.size());
}

TEST(roadgraph, FindPath_along_neighbor_edges_test) {
  using namespace bark::world::map;
  Roadgraph r;

  XodrLanePtr lane_0(new XodrLane());
  lane_0->SetId(10);
  lane_0->SetLaneType(XodrLaneType::DRIVING);
  XodrLanePtr lane_1(new XodrLane());
  lane_1->SetId(20);
  lane_1->SetLaneType(XodrLaneType::DRIVING);
  XodrLanePtr lane_2(new XodrLane());
  lane_2->SetId(30);
  lane_2->SetLaneType(XodrLaneType::DRIVING);
  XodrLaneId l0 = r.AddLane(0, lane_0);
  XodrLaneId l1 = r.AddLane(1, lane_1);
  XodrLaneId l2 = r.AddLane(2, lane_2);

  r.AddLaneSuccessor(l0, l1);
  r.AddInnerNeighbor(l1, l2);
  r.AddOuterNeighbor(l1, l2);

  // XodrLane l2 is not reachable using only successor edges, thus no valid path
  // available
  std::vector<XodrLaneId> path =
      r.FindPath<TypeDrivingAndEdgeTypeLaneSuccessor>(l0, l2);

  ASSERT_EQ(0, path.size());
}

TEST(roadgraph, get_driving_corridor_test) {
  using namespace bark::world::map;
  Roadgraph r;

  XodrRoadId rid0 = 0;
  XodrLanePtr lane_00(new XodrLane());
  XodrLanePtr lane_10(new XodrLane());
  XodrLaneId l00 = r.AddLane(rid0, lane_00);
  XodrLaneId l10 = r.AddLane(rid0, lane_10);

  XodrRoadId rid1 = 1;
  XodrLanePtr lane_01(new XodrLane());
  XodrLanePtr lane_11(new XodrLane());
  XodrLaneId l01 = r.AddLane(rid1, lane_01);
  XodrLaneId l11 = r.AddLane(rid1, lane_11);

  r.AddLaneSuccessor(l10, l11);
  r.AddInnerNeighbor(l00, l10);
  r.AddInnerNeighbor(l01, l11);

  // TODO(@all): add new assertions as this has moved to MapInterface
  /*
  std::pair< std::vector<XodrLanePtr>, std::vector<XodrLanePtr> > horizon =
  r.ComputeXodrLaneBoundariesHorizon(l10, l11);

  ASSERT_TRUE(horizon.first.size()==2);
  ASSERT_TRUE(horizon.second.size()==2);
  */
}

TEST(roadgraph, generate_roadgraph_neighbours_test) {
  using namespace bark::geometry;
  using namespace bark::world::opendrive;
  using namespace bark::world::map;

  OpenDriveMapPtr open_drive_map(new OpenDriveMap());
  //! ROAD 1
  PlanViewPtr p(new PlanView());
  p->AddLine(Point2d(0.0f, 0.0f), 0.0f, 10.0f);

  //! XodrLane-Section 1
  XodrLaneSectionPtr ls(new XodrLaneSection(0.0));

  //! XodrLane
  XodrLanePtr lane0(new XodrLane(0));
  lane0->SetLine(p->GetReferenceLine());

  XodrLaneOffset off = {1.0f, 0.0f, 0.0f, 0.0f};
  XodrLaneWidth lane_width_1 = {0, 10, off};

  XodrLanePtr lane1 =
      CreateLaneFromLaneWidth(-1, p->GetReferenceLine(), lane_width_1, 0.05);
  XodrLanePtr lane2 =
      CreateLaneFromLaneWidth(1, p->GetReferenceLine(), lane_width_1, 0.05);
  XodrLanePtr lane3 =
      CreateLaneFromLaneWidth(2, lane2->GetLine(), lane_width_1, 0.05);

  ls->AddLane(lane1);
  ls->AddLane(lane2);
  ls->AddLane(lane3);

  XodrRoadPtr r(new XodrRoad("highway", 100));
  r->SetPlanView(p);
  r->AddLaneSection(ls);

  open_drive_map->AddRoad(r);

  Roadgraph rg;
  rg.Generate(open_drive_map);

  std::pair<XodrLaneId, bool> in1 = rg.GetInnerNeighbor(lane1->GetId());
  // There is no inner neighbor for lane1, as lane1 and lane2 are left and right
  // of the planview
  ASSERT_FALSE(in1.second);

  std::pair<XodrLaneId, bool> out2 = rg.GetOuterNeighbor(lane2->GetId());
  ASSERT_TRUE(out2.second);
  ASSERT_TRUE(out2.first == lane3->GetId());

  // TODO: should use make_map_one_road_two_lanes
}

TEST(roadgraph, generate_roadgraph_successors_test) {
  using namespace bark::geometry;
  using namespace bark::world::opendrive;
  using namespace bark::world::map;

  OpenDriveMapPtr open_drive_map(new OpenDriveMap());
  //! ROAD 1
  PlanViewPtr p(new PlanView());
  p->AddLine(Point2d(0.0f, 0.0f), 0.0f, 10.0f);

  //! XodrLane-Section 1
  XodrLaneSectionPtr ls(new XodrLaneSection(0.0));

  //! XodrLane
  XodrLanePtr lane00(new XodrLane(0));
  lane00->SetLine(p->GetReferenceLine());

  XodrLaneOffset off = {1.0f, 0.0f, 0.0f, 0.0f};
  XodrLaneWidth lane_width_1 = {0, 10, off};
  XodrLanePtr lane10 =
      CreateLaneFromLaneWidth(1, p->GetReferenceLine(), lane_width_1, 0.05);
  XodrLaneLink ll1;
  ll1.from_position = 1;
  ll1.to_position = 1;
  lane10->SetLink(ll1);

  ls->AddLane(lane00);
  ls->AddLane(lane10);

  XodrRoadPtr r(new XodrRoad("highway", 100));
  r->SetPlanView(p);
  r->AddLaneSection(ls);

  //! ROAD 2
  PlanViewPtr p2(new PlanView());
  p2->AddLine(Point2d(10.0f, 0.0f), 0.0f, 20.0f);

  //! XodrLane-Section 2
  XodrLaneSectionPtr ls2(new XodrLaneSection(0.0));

  //! XodrLane
  XodrLanePtr lane01(new XodrLane(0));
  lane01->SetLine(p2->GetReferenceLine());

  XodrLaneOffset off2 = {1.0f, 0.0f, 0.0f, 0.0f};
  XodrLaneWidth lane_width_2 = {0, 10, off};
  XodrLanePtr lane11 =
      CreateLaneFromLaneWidth(1, p2->GetReferenceLine(), lane_width_2, 0.05);
  XodrLaneLink ll2;
  ll2.from_position = 1;
  ll2.to_position = 1;
  lane11->SetLink(ll2);
  ls2->AddLane(lane01);
  ls2->AddLane(lane11);

  XodrRoadPtr r2(new XodrRoad("highway", 101));
  r2->SetPlanView(p2);
  r2->AddLaneSection(ls2);

  XodrRoadLinkInfo rli_succ(r2->GetId(), "road");
  XodrRoadLink rl;
  rl.SetSuccessor(rli_succ);
  r->SetLink(rl);

  open_drive_map->AddRoad(r);
  open_drive_map->AddRoad(r2);

  Roadgraph rg;
  rg.Generate(open_drive_map);

  rg.PrintGraph("generate_roadgraph_successors_test.dot");

  std::vector<XodrLaneId> suc = rg.GetSuccessorLanes(lane10->GetId());
  std::cout << "suc: " << suc.size() << std::endl;
  ASSERT_TRUE(suc.size() == 1);
  ASSERT_TRUE(suc[0] == lane11->GetId());

  auto all_ids = rg.GetAllLaneids();
  ASSERT_TRUE(all_ids.size() == 2);
}

TEST(roadgraph, MakeXodrMapOneRoadTwoLanes_test) {
  using namespace bark::geometry;
  using namespace bark::world::opendrive;
  using namespace bark::world::map;

  OpenDriveMapPtr open_drive_map =
      bark::world::tests::MakeXodrMapOneRoadTwoLanes();
  Roadgraph rg;
  rg.Generate(open_drive_map);

  rg.PrintGraph("MakeXodrMapOneRoadTwoLanes_test.dot");

  auto all_ids = rg.GetAllLaneids();
  ASSERT_TRUE(all_ids.size() == 2);
}

TEST(roadgraph, MakeXodrMapTwoRoadsOneLane_test) {
  using namespace bark::geometry;
  using namespace bark::world::opendrive;
  using namespace bark::world::map;

  OpenDriveMapPtr open_drive_map =
      bark::world::tests::MakeXodrMapTwoRoadsOneLane();
  Roadgraph rg;
  rg.Generate(open_drive_map);

  rg.PrintGraph("MakeXodrMapTwoRoadsOneLane_test.dot");

  auto all_ids = rg.GetAllLaneids();
  ASSERT_TRUE(all_ids.size() == 2);
}