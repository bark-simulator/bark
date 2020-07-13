// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/world/map/roadgraph.hpp"
#include <memory>
#include <utility>

namespace bark {
namespace world {
namespace map {

XodrLaneId Roadgraph::AddLane(const XodrRoadId& road_id,
                              const XodrLanePtr& laneptr) {
  XodrLaneVertex lane = XodrLaneVertex(road_id, laneptr->GetId(), laneptr);
  boost::add_vertex(lane, g_);
  return laneptr->GetId();
}

bool Roadgraph::AddInnerNeighbor(const XodrLaneId& inner_id,
                                 const XodrLaneId& outer_id) {
  return AddEdgeOfType(outer_id, inner_id, INNER_NEIGHBOR_EDGE);
}

bool Roadgraph::AddOuterNeighbor(const XodrLaneId& inner_id,
                                 const XodrLaneId& outer_id) {
  return AddEdgeOfType(inner_id, outer_id, OUTER_NEIGHBOR_EDGE);
}

bool Roadgraph::AddLaneSuccessor(const XodrLaneId& prev,
                                 const XodrLaneId& succ) {
  return AddEdgeOfType(prev, succ, LANE_SUCCESSOR_EDGE);
}

bool Roadgraph::AddRoadSuccessor(const XodrLaneId& prev,
                                 const XodrLaneId& succ) {
  return AddEdgeOfType(prev, succ, ROAD_SUCCESSOR_EDGE);
}

std::vector<XodrLaneId> Roadgraph::GetSuccessorLanes(
    const XodrLaneId& lane_id) const {
  std::pair<vertex_t, bool> lane_vertex_pair = GetVertexByLaneId(lane_id);
  boost::graph_traits<XodrLaneGraph>::out_edge_iterator i, end;
  std::vector<XodrLaneId> successor_lanes;
  for (boost::tie(i, end) = boost::out_edges(lane_vertex_pair.first, g_);
       i != end; ++i) {
    if (g_[*i].edge_type == LANE_SUCCESSOR_EDGE) {
      vertex_t target = boost::target(*i, g_);
      successor_lanes.push_back(g_[target].global_lane_id);
    }
  }
  return successor_lanes;
}

std::vector<XodrLaneId> Roadgraph::GetPredecessorLanes(
    const XodrLaneId& lane_id) const {
  std::pair<vertex_t, bool> lane_vertex_pair = GetVertexByLaneId(lane_id);
  boost::graph_traits<XodrLaneGraph>::in_edge_iterator i, end;
  std::vector<XodrLaneId> predecessor_lanes;
  for (boost::tie(i, end) = boost::in_edges(lane_vertex_pair.first, g_);
       i != end; ++i) {
    if (g_[*i].edge_type == LANE_SUCCESSOR_EDGE) {
      vertex_t source = boost::source(*i, g_);
      predecessor_lanes.push_back(g_[source].global_lane_id);
    }
  }
  return predecessor_lanes;
}

std::vector<XodrRoadId> Roadgraph::FindRoadPath(const XodrRoadId& startid,
                                                const XodrRoadId& goalid) {
  auto start_pv = GetPlanViewForRoadId(startid);
  auto goal_pv = GetPlanViewForRoadId(goalid);

  std::vector<XodrRoadId> road_ids;

  if (start_pv.second && goal_pv.second) {
    std::vector<XodrLaneId> lane_ids =
        FindPath<EdgeTypeRoadSuccessor>(start_pv.first, goal_pv.first);

    for (auto const& id : lane_ids) {
      road_ids.push_back(GetRoadForLaneId(id));
    }
  }
  return road_ids;
}

std::vector<XodrLaneId> Roadgraph::FindDrivableLanePath(
    const XodrLaneId& startid, const XodrLaneId& goalid) {
  return FindPath<TypeDriving>(startid, goalid);
}

std::vector<std::vector<XodrLaneId>> Roadgraph::FindAllPathsInSubgraph(
    const std::vector<XodrLaneEdgeType>& edge_type_subset,
    const std::vector<XodrLaneId>& lane_id_subset) {
  struct Predicate {
    bool operator()(XodrLaneGraph::edge_descriptor edge_des) const {
      XodrLaneEdgeType lane_edge_type = (*g)[edge_des].GetEdgeType();
      return (find(edge_type_subset.begin(), edge_type_subset.end(),
                   lane_edge_type) != edge_type_subset.end());
    }
    bool operator()(XodrLaneGraph::vertex_descriptor vertex_des) const {
      XodrLaneId lane_id = (*g)[vertex_des].global_lane_id;
      return (find(lane_id_subset.begin(), lane_id_subset.end(), lane_id) !=
              lane_id_subset.end());
    }

    const XodrLaneGraph* g;
    std::vector<XodrLaneEdgeType> edge_type_subset;
    std::vector<XodrLaneId> lane_id_subset;
  } predicate{&g_, edge_type_subset, lane_id_subset};
  boost::filtered_graph<XodrLaneGraph, Predicate, Predicate> filtered_graph(
      g_, predicate, predicate);

  std::vector<std::vector<XodrLaneId>> paths;

  for (vertex_t const& start_vertex :
       boost::make_iterator_range(vertices(filtered_graph))) {
    if (in_degree(start_vertex, filtered_graph) > 0) {
      continue;
    }
    // start_vertex has no predecessors
    std::vector<vertex_t> p(boost::num_vertices(filtered_graph));
    std::vector<int> d(boost::num_vertices(filtered_graph));
    boost::property_map<XodrLaneGraph, float XodrLaneEdge::*>::type weightmap =
        boost::get(&XodrLaneEdge::weight, filtered_graph);

    boost::dijkstra_shortest_paths(
        filtered_graph, start_vertex,
        predecessor_map(
            boost::make_iterator_property_map(
                p.begin(), get(boost::vertex_index, filtered_graph)))
            .distance_map(boost::make_iterator_property_map(
                d.begin(), get(boost::vertex_index, filtered_graph)))
            .weight_map(weightmap));
    for (auto const& goal_vertex :
         boost::make_iterator_range(vertices(filtered_graph))) {
      if (out_degree(goal_vertex, filtered_graph) > 0) {
        continue;
      }
      // goal_vertex has no successors
      std::vector<XodrLaneId> path;
      bool reachable = true;
      boost::graph_traits<XodrLaneGraph>::vertex_descriptor current =
          goal_vertex;
      while (current != start_vertex) {
        path.push_back(filtered_graph[current].global_lane_id);
        if (p[current] == current) {
          reachable = false;
          break;
        }
        current = p[current];
      }
      if (reachable) {
        path.push_back(filtered_graph[start_vertex].global_lane_id);
        std::reverse(path.begin(), path.end());
        paths.push_back(path);
      }
    }
  }
  return paths;
}

XodrLanePtr Roadgraph::GetLanePtr(const XodrLaneId& id) const {
  std::pair<vertex_t, bool> lane_vertex_pair = GetVertexByLaneId(id);
  if (lane_vertex_pair.second) {
    return g_[lane_vertex_pair.first].lane;
  } else {
    return NULL;
  }
}

std::vector<XodrLaneId> Roadgraph::GetAllLaneids() const {
  std::vector<XodrLaneId> ids;
  std::vector<vertex_t> vertices = GetVertices();
  for (auto const& v : vertices) {
    if (GetLaneGraph()[v].lane->GetLanePosition() != 0) {
      ids.push_back(GetLaneGraph()[v].global_lane_id);
    }
  }
  return ids;
}

std::pair<XodrLaneId, bool> Roadgraph::GetPlanViewForRoadId(
    const XodrRoadId& id) const {
  std::vector<XodrLaneId> ids;
  std::vector<vertex_t> vertices = GetVertices();
  for (auto const& v : vertices) {
    if (GetLaneGraph()[v].road_id == id) {
      if (GetLaneGraph()[v].lane->GetLanePosition() == 0) {
        return std::make_pair(GetLaneGraph()[v].global_lane_id, true);
      }
    }
  }
  return std::make_pair(0, false);
}

std::pair<std::vector<XodrDrivingDirection>, bool>
Roadgraph::GetDrivingDirectionsForRoadId(const XodrRoadId& id) const {
  std::vector<XodrDrivingDirection> directions;
  std::vector<vertex_t> vertices = GetVertices();
  bool road_found = false;
  for (auto const& v : vertices) {
    if (GetLaneGraph()[v].road_id == id) {
      directions.push_back(GetLaneGraph()[v].lane->GetDrivingDirection());
    }
  }
  return std::make_pair(directions, road_found);
}

std::pair<XodrLaneId, bool> Roadgraph::GetPlanViewForLaneId(
    const XodrLaneId& outer_lane_id) const {
  XodrLanePtr lane = GetLanePtr(outer_lane_id);
  // std::cout << "calling GetPlanViewForLaneId() with " << outer_lane_id <<
  // std::endl;
  if (lane->GetLanePosition() == 0) {
    return std::make_pair(outer_lane_id, true);
  } else {
    // std::cout << outer_lane_id << " is not the planview " << lane->GetId()
    // << std::endl;
    std::pair<XodrLaneId, bool> inner_lane = GetInnerNeighbor(lane->GetId());
    // std::cout << "called GetInnerNeighbor: " << inner_lane.first <<
    // inner_lane.second << std::endl;
    if (inner_lane.second) {
      return GetPlanViewForLaneId(inner_lane.first);
    } else {
      // std::cout << "there was an error !! " << inner_lane.first << std::endl;
      // their appears to be some error
      return std::make_pair(0, false);
    }
  }
}

PolygonPtr Roadgraph::GetLanePolygonForLaneId(const XodrLaneId& lane_id) {
  auto v = GetVertexByLaneId(lane_id);
  return GetLaneGraph().operator[](v.first).polygon;
}

XodrRoadId Roadgraph::GetRoadForLaneId(const XodrLaneId& lane_id) {
  auto v = GetVertexByLaneId(lane_id);
  return GetLaneGraph().operator[](v.first).road_id;
}

std::pair<XodrLaneId, bool> Roadgraph::GetInnerNeighbor(
    const XodrLaneId& lane_id) const {
  std::vector<std::pair<XodrLaneId, bool>> neighbors =
      GetNeighborFromEdgetype(lane_id, INNER_NEIGHBOR_EDGE);
  std::pair<XodrLaneId, bool> neighbor =
      neighbors.front();  // inner neighbor is unique
  return neighbor;
}

std::pair<XodrLaneId, bool> Roadgraph::GetOuterNeighbor(
    const XodrLaneId& lane_id) const {
  std::vector<std::pair<XodrLaneId, bool>> tmp =
      GetNeighborFromEdgetype(lane_id, OUTER_NEIGHBOR_EDGE);
  return tmp.front();  // for a non planview edge the outer neighbor is unique
}

std::vector<std::pair<XodrLaneId, bool>> Roadgraph::GetOuterNeighborsPlanview(
    const XodrLaneId& lane_id) const {
  std::vector<std::pair<XodrLaneId, bool>> tmp =
      GetNeighborFromEdgetype(lane_id, OUTER_NEIGHBOR_EDGE);
  return tmp;
}

std::vector<XodrLaneId> Roadgraph::GetAllNeighbors(
    const XodrLaneId& lane_id) const {
  XodrLanePtr lane = GetLanePtr(lane_id);
  if (lane->GetLanePosition() == 0) {
    throw std::runtime_error("GetAllNeighbors was called with the plan view");
  }

  std::vector<XodrLaneId> neighbors;

  std::pair<XodrLaneId, bool> current_neighbor = GetInnerNeighbor(lane_id);
  std::pair<XodrLaneId, bool> next_neighbor = std::make_pair(0, false);

  // Inner neighbors
  if (current_neighbor.second) {
    // We do not want to add the planview to the vector of neighbors, therefore
    // check if the current neighbor has an inner neighbor
    next_neighbor = GetInnerNeighbor(current_neighbor.first);
  }
  while (next_neighbor.second) {
    neighbors.push_back(current_neighbor.first);
    current_neighbor = next_neighbor;
    next_neighbor = GetInnerNeighbor(current_neighbor.first);
  }

  // Outer neighbors
  current_neighbor = GetOuterNeighbor(lane_id);
  while (current_neighbor.second) {
    neighbors.push_back(current_neighbor.first);
    current_neighbor = GetOuterNeighbor(current_neighbor.first);
  }

  return neighbors;
}

bool Roadgraph::HasLane(const XodrLaneId& lane_id) const {
  boost::graph_traits<XodrLaneGraph>::vertex_iterator i, end;
  for (boost::tie(i, end) = boost::vertices(g_); i != end; ++i) {
    if (g_[*i].global_lane_id == lane_id) {
      return true;
    }
  }
  return false;
}

void Roadgraph::PrintGraph(const char* filename) {
  std::ofstream dotfile(filename);
  PrintGraph(dotfile);
  dotfile.close();
}

void Roadgraph::PrintGraph(std::ofstream& dotfile) {
  write_graphviz(
      dotfile, g_,
      make_vertex_label_writer_graph(
          boost::get(&XodrLaneVertex::road_id, g_),
          boost::get(&XodrLaneVertex::global_lane_id, g_),
          boost::get(&XodrLaneVertex::lane, g_)),
      make_edge_label_writer_text(get(&XodrLaneEdge::edge_type, g_)));
}

std::vector<vertex_t> Roadgraph::GetVertices() const {
  std::vector<vertex_t> vertex_descriptors;
  boost::graph_traits<XodrLaneGraph>::vertex_iterator i, end;
  for (boost::tie(i, end) = boost::vertices(g_); i != end; ++i) {
    vertex_descriptors.push_back(*i);
  }
  return vertex_descriptors;
}

std::vector<vertex_t> Roadgraph::GetNextVertices(
    vertex_t current_vertex) const {
  std::vector<vertex_t> vertex_descriptors;
  boost::graph_traits<XodrLaneGraph>::out_edge_iterator ei, ei_end;
  for (boost::tie(ei, ei_end) = boost::out_edges(current_vertex, g_);
       ei != ei_end; ei++) {
    vertex_t target = boost::target(*ei, g_);
    vertex_descriptors.push_back(target);
  }
  return vertex_descriptors;
}

std::vector<edge_t> Roadgraph::GetOutEdges(vertex_t current_vertex) const {
  std::vector<edge_t> edge_descriptors;
  boost::graph_traits<XodrLaneGraph>::out_edge_iterator ei, ei_end;
  for (boost::tie(ei, ei_end) = boost::out_edges(current_vertex, g_);
       ei != ei_end; ei++) {
    vertex_t source = boost::source(*ei, g_);
    vertex_t target = boost::target(*ei, g_);
    auto edge = boost::edge(source, target, g_);
    edge_descriptors.push_back(edge.first);
  }
  return edge_descriptors;
}

std::vector<edge_t> Roadgraph::GetEdges() const {
  std::vector<edge_t> edge_descriptors;
  boost::graph_traits<XodrLaneGraph>::edge_iterator ei, ei_end;
  for (boost::tie(ei, ei_end) = boost::edges(g_); ei != ei_end; ei++) {
    vertex_t source = boost::source(*ei, g_);
    vertex_t target = boost::target(*ei, g_);
    auto edge = boost::edge(source, target, g_);
    edge_descriptors.push_back(edge.first);
  }
  return edge_descriptors;
}

edge_t Roadgraph::GetEdgeDescr(vertex_t from, vertex_t to) const {
  return boost::edge(from, to, g_).first;
}

std::pair<vertex_t, bool> Roadgraph::GetVertexByLaneId(
    const XodrLaneId& lane_id) const {
  std::pair<vertex_t, bool> retval;
  retval.second = false;
  boost::graph_traits<XodrLaneGraph>::vertex_iterator i, end;
  for (boost::tie(i, end) = boost::vertices(g_); i != end; ++i) {
    if (g_[*i].global_lane_id == lane_id) {
      retval.first = *i;
      retval.second = true;
      break;
    }
  }
  return retval;
}

void Roadgraph::GenerateVertices(OpenDriveMapPtr map) {
  for (auto const& road_element : map->GetRoads()) {  // std::map
    for (auto const& lane_section_element :
         road_element.second->GetLaneSections()) {  // std::vector
      for (auto const& lane_element :
           lane_section_element->GetLanes()) {  // std::map
        AddLane(road_element.first, lane_element.second);
      }
    }
  }
}

void Roadgraph::GeneratePreAndSuccessors(OpenDriveMapPtr map) {
  // add successors, predecessors
  for (auto const& road_element : map->GetRoads()) {  // std::map
    XodrRoadId successor_id = road_element.second->GetLink()
                                  .GetSuccessor()
                                  .id_;  // this is the position!!!!!! (-4, 4)
    XodrRoadId predecessor_id = road_element.second->GetLink()
                                    .GetPredecessor()
                                    .id_;  // this is the position!!!!!! (-4, 4)
    if (successor_id > 1000) {
      continue;
    }

    // make sure that there is a predecessor!!
    XodrLaneSectionPtr successor_lane_section = nullptr;
    XodrLaneSectionPtr predecessor_lane_section = nullptr;

    // TODO(@hart): That's pretty ugly, move check for road_id to map

    if (road_element.second->GetLink().GetSuccessor().type_ ==
        "road") {  // successor_id == iter->first
      auto it = map->GetRoads().find(successor_id);
      if (it != map->GetRoads().end()) {
        XodrRoadPtr successor_road = map->GetRoads().at(successor_id);
        successor_lane_section = successor_road->GetLaneSections().front();
      }
    } else {
      LOG(INFO) << "XodrRoad has no successor road. \n";
    }

    if (road_element.second->GetLink().GetPredecessor().type_ ==
        "road") {  // predecessor_id == iter->first
      auto it = map->GetRoads().find(predecessor_id);
      if (it != map->GetRoads().end()) {
        XodrRoadPtr predecessor_road = map->GetRoads().at(predecessor_id);
        predecessor_lane_section = predecessor_road->GetLaneSections().back();
      }
    } else {
      LOG(INFO) << "XodrRoad has no predeseccor road. \n";
    }

    // TODO(@hart): there could be mult. lane_sections
    for (auto const& lane_section_element :
         road_element.second->GetLaneSections()) {  // std::vector
      for (auto const& lane_element :
           lane_section_element->GetLanes()) {  // std::map
        // add successor edge
        if (successor_lane_section) {
          XodrLanePosition successor_lane_position =
              lane_element.second->GetLink().to_position;

          if (successor_lane_position == 0) {
            continue;
          }

          XodrLanePtr successor_lane =
              successor_lane_section->GetLaneByPosition(
                  successor_lane_position);

          if (successor_lane) {
            bool success =
                AddLaneSuccessor(lane_element.first, successor_lane->GetId());
            success =
                AddLaneSuccessor(successor_lane->GetId(), lane_element.first);
            auto lane_pv = GetPlanViewForLaneId(lane_element.first);
            auto succ_pv = GetPlanViewForLaneId(successor_lane->GetId());
            BARK_EXPECT_TRUE(lane_pv.second && succ_pv.second);
            AddRoadSuccessor(lane_pv.first, succ_pv.first);
            AddRoadSuccessor(succ_pv.first, lane_pv.first);
          }
        }

        // does not always have predecessor
        try {
          // search for predecessor_lane_position in previos lane section
          if (predecessor_lane_section) {
            XodrLanePosition predecessor_lane_position =
                lane_element.second->GetLink().from_position;

            if (predecessor_lane_position == 0) {
              continue;
            }

            XodrLanePtr predecessor_lane =
                predecessor_lane_section->GetLaneByPosition(
                    predecessor_lane_position);

            if (predecessor_lane) {
              // if found add; convert predecessor to successor
              bool success = AddLaneSuccessor(predecessor_lane->GetId(),
                                              lane_element.first);
              success = AddLaneSuccessor(lane_element.first,
                                         predecessor_lane->GetId());
              auto lane_pv = GetPlanViewForLaneId(lane_element.first);
              auto pred_pv = GetPlanViewForLaneId(predecessor_lane->GetId());
              BARK_EXPECT_TRUE(lane_pv.second && pred_pv.second);
              AddRoadSuccessor(pred_pv.first, lane_pv.first);
              AddRoadSuccessor(lane_pv.first, pred_pv.first);
            }
          }
        } catch (const std::exception& ex) {
          LOG(INFO) << "XodrRoad has no predeseccor road. \n";
        }
      }
    }
  }
}

void Roadgraph::GenerateNeighbours(OpenDriveMapPtr map) {
  // add neighbor edges
  for (auto const& road_element : map->GetRoads()) {  // std::map
    for (auto const& lane_section_element :
         road_element.second->GetLaneSections()) {  // std::vector
      for (auto const& lane_element :
           lane_section_element->GetLanes()) {  // std::map
        if (lane_element.second->GetLanePosition() != 0) {
          XodrLanePosition inner_lane_pos;
          if (lane_element.second->GetLanePosition() > 0) {
            inner_lane_pos = lane_element.second->GetLanePosition() - 1;
          } else {
            inner_lane_pos = lane_element.second->GetLanePosition() + 1;
          }

          XodrLanePtr inner_lane =
              lane_section_element->GetLaneByPosition(inner_lane_pos);
          if (inner_lane) {
            bool success_inner = AddInnerNeighbor(inner_lane->GetId(),
                                                  lane_element.second->GetId());
            bool success_outer = AddOuterNeighbor(inner_lane->GetId(),
                                                  lane_element.second->GetId());
          }
        }
      }
    }
  }
}

void Roadgraph::GenerateFromJunctions(OpenDriveMapPtr map) {
  // map junctions
  for (auto const& road_element : map->GetJunctions()) {  // std::map
    for (auto const& connection_element :
         road_element.second->GetConnections()) {  // std::map
      XodrRoadPtr incoming_road =
          map->GetRoad(connection_element.second.incoming_road_);
      XodrRoadPtr connecting_road =
          map->GetRoad(connection_element.second.connecting_road_);
      XodrLaneSectionPtr pre_lane_section =
          incoming_road->GetLaneSections().front();
      XodrLaneSectionPtr successor_lane_section =
          connecting_road->GetLaneSections().front();
      for (auto const& lane_link_element :
           connection_element.second.GetLaneLinks()) {
        // add successor edge
        if (pre_lane_section && successor_lane_section) {
          try {
            XodrLanePtr pre_lane = pre_lane_section->GetLaneByPosition(
                lane_link_element.from_position);
            XodrLanePtr successor_lane =
                successor_lane_section->GetLaneByPosition(
                    lane_link_element.to_position);
            if (pre_lane && successor_lane) {
              bool success =
                  AddLaneSuccessor(pre_lane->GetId(), successor_lane->GetId());
              success =
                  AddLaneSuccessor(successor_lane->GetId(), pre_lane->GetId());
              // also connect road elements (through plan view)
              auto pre_plan_view = GetPlanViewForLaneId(pre_lane->GetId());
              auto succ_plan_view =
                  GetPlanViewForLaneId(successor_lane->GetId());
              success =
                  AddRoadSuccessor(pre_plan_view.first, succ_plan_view.first);
              success =
                  AddRoadSuccessor(succ_plan_view.first, pre_plan_view.first);
            }
          } catch (...) {
            LOG(INFO) << "Junction has no connections. \n";
          }
        }
      }
    }
  }
}

void Roadgraph::GeneratePolygonsForVertices() {
  std::vector<vertex_t> vertices = GetVertices();
  for (auto const& v : vertices) {
    // g_[v].polygon =
    // ComputeXodrLanePolygon(GetLaneGraph()[v].lane->GetId());
    if (GetLaneGraph()[v].lane->GetLanePosition() != 0) {
      auto p = ComputeXodrLanePolygon(GetLaneGraph()[v].lane->GetId());
      if (p.second) {
        g_[v].polygon = p.first;
      }
    }
    // auto l = GetLaneGraph()[v].lane->GetId();
  }
}

void Roadgraph::Generate(OpenDriveMapPtr map) {
  GenerateVertices(map);

  GenerateNeighbours(map);

  GeneratePreAndSuccessors(map);

  GenerateFromJunctions(map);

  GeneratePolygonsForVertices();
}

std::pair<XodrLanePtr, XodrLanePtr> Roadgraph::ComputeXodrLaneBoundaries(
    const XodrLaneId& lane_id) const {
  XodrLanePtr inner, outer;
  std::pair<vertex_t, bool> v = GetVertexByLaneId(lane_id);
  auto l = GetLaneGraph()[v.first].lane;
  // assert(l->GetLanePosition() != 0); // make sure we are not at the
  // planview, as a driving corridor cannot be computed from here.
  outer = l;

  std::pair<XodrLaneId, bool> innerid = GetInnerNeighbor(lane_id);
  if (innerid.second) {
    std::pair<vertex_t, bool> v_inner = GetVertexByLaneId(innerid.first);
    if (v_inner.second) {
      inner = GetLaneGraph()[v_inner.first].lane;
    } else {
      inner = NULL;
    }
  } else {  // you are probably at the planview and do not have inner lanes?
  }
  return std::make_pair(inner, outer);
}

std::pair<std::vector<XodrLanePtr>, std::vector<XodrLanePtr>>
Roadgraph::ComputeRouteBoundaries(
    const std::vector<XodrLaneId>& horizon) const {
  std::vector<XodrLanePtr> inner, outer;
  if (!horizon.empty()) {
    for (auto& h : horizon) {
      auto lane_boundaries = ComputeXodrLaneBoundaries(h);
      inner.push_back(lane_boundaries.first);
      outer.push_back(lane_boundaries.second);
    }
  }
  return std::make_pair(inner, outer);
}

std::pair<PolygonPtr, bool> Roadgraph::ComputeXodrLanePolygon(
    const XodrLaneId& lane_id) const {
  bool success = false;
  std::pair<XodrLanePtr, XodrLanePtr> lb = ComputeXodrLaneBoundaries(lane_id);

  PolygonPtr polygon = std::make_shared<bark::geometry::Polygon>();
  // inner
  if (lb.first && lb.second) {
    success = true;

    for (auto const& p : lb.first->GetLine()) {
      polygon->AddPoint(p);
    }
    // outer
    auto reversed_outer = lb.second->GetLine();
    reversed_outer.Reverse();

    for (auto const& p : reversed_outer) {
      polygon->AddPoint(p);
    }
    // Polygons need to be closed!
    polygon->AddPoint(*(lb.first->GetLine().begin()));
  }
  return std::make_pair(polygon, success);
}

bool Roadgraph::AddEdgeOfType(const XodrLaneId& source_id,
                              const XodrLaneId& tarGetId,
                              const XodrLaneEdgeType& edgetype) {
  XodrLaneEdge edge = XodrLaneEdge(edgetype);
  std::pair<vertex_t, bool> source_lane = GetVertexByLaneId(source_id);
  std::pair<vertex_t, bool> tarGetLane = GetVertexByLaneId(tarGetId);
  if (source_lane.second && tarGetLane.second) {
    boost::add_edge(source_lane.first, tarGetLane.first, edge, g_);
    return true;
  } else {
    return false;
  }
}

std::vector<std::pair<XodrLaneId, bool>> Roadgraph::GetNeighborFromEdgetype(
    const XodrLaneId& lane_id, const XodrLaneEdgeType edge_type)
    const {  //! we can have two outer neighbors
  std::vector<std::pair<XodrLaneId, bool>> retval;
  std::pair<vertex_t, bool> lane_vertex_pair = GetVertexByLaneId(lane_id);
  boost::graph_traits<XodrLaneGraph>::out_edge_iterator i, end;
  for (boost::tie(i, end) = boost::out_edges(lane_vertex_pair.first, g_);
       i != end; ++i) {
    if (g_[*i].edge_type == edge_type) {
      vertex_t target = boost::target(*i, g_);
      retval.push_back(std::make_pair(g_[target].global_lane_id, true));
    }
  }
  if (retval.empty()) {
    retval.push_back(std::make_pair(0, false));
  }
  return retval;
}

std::pair<XodrLaneId, bool> Roadgraph::GetNextLane(
    const std::vector<XodrRoadId>& road_ids, const XodrLaneId& lane_id) {
  std::pair<vertex_t, bool> v = GetVertexByLaneId(lane_id);
  // XodrRoadId roadid = GetLaneGraph().operator[](v.first).road_id;
  boost::graph_traits<XodrLaneGraph>::out_edge_iterator i, end;
  for (boost::tie(i, end) = boost::out_edges(v.first, g_); i != end; ++i) {
    if (g_[*i].edge_type == LANE_SUCCESSOR_EDGE) {
      vertex_t v_target = boost::target(*i, g_);
      XodrRoadId road_id_next = GetLaneGraph().operator[](v_target).road_id;
      if (std::find(road_ids.begin(), road_ids.end(), road_id_next) !=
          road_ids.end()) {
        return std::make_pair(g_[v_target].global_lane_id, true);
      }
    }
  }
  return std::make_pair(0, false);
}

std::pair<XodrLaneId, bool> Roadgraph::GetLeftLane(
    const XodrLaneId& lane_id, const XodrDrivingDirection& driving_direction) {
  XodrLanePtr lane = GetLanePtr(lane_id);
  if (driving_direction == lane->GetDrivingDirection()) {
    std::pair<XodrLaneId, bool> inner_neighbor = GetInnerNeighbor(lane_id);
    if (inner_neighbor.second) {
      XodrLanePtr inner_lane = GetLanePtr(inner_neighbor.first);
      if (inner_lane->GetLanePosition() != 0)
        return std::make_pair(inner_neighbor.first, true);
      std::vector<std::pair<XodrLaneId, bool>> outer_neighbors =
          GetOuterNeighborsPlanview(inner_neighbor.first);
      for (auto& outer_neighbor : outer_neighbors)
        if (outer_neighbor.second && outer_neighbor.first != lane_id)
          return std::make_pair(outer_neighbor.first, true);
    }
  } else {
    std::pair<XodrLaneId, bool> outer_neighbor = GetOuterNeighbor(lane_id);
    if (outer_neighbor.second)
      return std::make_pair(outer_neighbor.first, true);
  }
  return std::make_pair(0, false);
}

std::pair<XodrLaneId, bool> Roadgraph::GetRightLane(
    const XodrLaneId& lane_id, const XodrDrivingDirection& driving_direction) {
  // if (forwards and negative)  -> get_outer()
  // if (forwards and positive)  -> get_inner()
  //   if (inner_id == 0) -> get_outer()

  // if (backwards and positive) -> outer
  // if (backwards and negative) -> inner
  //   if (inner_id == 0) -> get_outer()
  XodrLanePtr lane = GetLanePtr(lane_id);
  if (driving_direction == lane->GetDrivingDirection()) {
    std::pair<XodrLaneId, bool> outer_neighbor = GetOuterNeighbor(lane_id);
    if (outer_neighbor.second)
      return std::make_pair(outer_neighbor.first, true);
  } else {
    std::pair<XodrLaneId, bool> inner_neighbor = GetInnerNeighbor(lane_id);
    if (inner_neighbor.second) {
      XodrLanePtr inner_lane = GetLanePtr(inner_neighbor.first);
      if (inner_lane->GetLanePosition() != 0)
        return std::make_pair(inner_neighbor.first, true);
      std::vector<std::pair<XodrLaneId, bool>> outer_neighbors =
          GetOuterNeighborsPlanview(inner_neighbor.first);
      for (auto& outer_neighbor : outer_neighbors)
        if (outer_neighbor.second && outer_neighbor.first != lane_id)
          return std::make_pair(outer_neighbor.first, true);
    }
  }
  return std::make_pair(0, false);
}

std::pair<XodrLaneId, bool> Roadgraph::GetLeftBoundary(
    const XodrLaneId& lane_id, const XodrDrivingDirection& driving_direction) {
  XodrLanePtr lane = GetLanePtr(lane_id);
  if (driving_direction == lane->GetDrivingDirection()) {
    std::pair<XodrLaneId, bool> inner_neighbor = GetInnerNeighbor(lane_id);
    if (inner_neighbor.second)
      return std::make_pair(inner_neighbor.first, true);
  } else {
    return std::make_pair(lane_id, true);
  }
  return std::make_pair(0, false);
}

std::pair<XodrLaneId, bool> Roadgraph::GetRightBoundary(
    const XodrLaneId& lane_id, const XodrDrivingDirection& driving_direction) {
  XodrLanePtr lane = GetLanePtr(lane_id);
  if (driving_direction == lane->GetDrivingDirection()) {
    return std::make_pair(lane_id, true);
  } else {
    std::pair<XodrLaneId, bool> inner_neighbor = GetInnerNeighbor(lane_id);
    if (inner_neighbor.second)
      return std::make_pair(inner_neighbor.first, true);
  }
  return std::make_pair(0, false);
}

}  // namespace map
}  // namespace world
}  // namespace bark
