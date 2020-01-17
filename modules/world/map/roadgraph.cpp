// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/world/map/roadgraph.hpp"
#include <memory>
#include <utility>

namespace modules {
namespace world {
namespace map {

XodrLaneId Roadgraph::add_lane(const XodrRoadId &road_id, const XodrLanePtr &laneptr) {
  XodrLaneVertex lane = XodrLaneVertex(road_id, laneptr->get_id(), laneptr);
  boost::add_vertex(lane, g_);
  return laneptr->get_id();
}

bool Roadgraph::add_inner_neighbor(const XodrLaneId &inner_id,
                                   const XodrLaneId &outer_id) {
  return add_edge_of_type(outer_id, inner_id, INNER_NEIGHBOR_EDGE);
}

bool Roadgraph::add_outer_neighbor(const XodrLaneId &inner_id,
                                   const XodrLaneId &outer_id) {
  return add_edge_of_type(inner_id, outer_id, OUTER_NEIGHBOR_EDGE);
}

bool Roadgraph::add_lane_successor(const XodrLaneId &prev, const XodrLaneId &succ) {
  return add_edge_of_type(prev, succ, LANE_SUCCESSOR_EDGE);
}

bool Roadgraph::add_road_successor(const XodrLaneId &prev, const XodrLaneId &succ) {
  return add_edge_of_type(prev, succ, ROAD_SUCCESSOR_EDGE);
}

std::vector<XodrLaneId> Roadgraph::get_successor_lanes(
    const XodrLaneId &lane_id) const {
  std::pair<vertex_t, bool> lane_vertex_pair = get_vertex_by_lane_id(lane_id);
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

std::vector<XodrLaneId> Roadgraph::get_predecessor_lanes(
    const XodrLaneId &lane_id) const {
  std::pair<vertex_t, bool> lane_vertex_pair = get_vertex_by_lane_id(lane_id);
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

XodrRoadId Roadgraph::GetNextRoad(const XodrRoadId& road_id) const {
  // TODO(@Klemens)

  XodrRoadId id = 0;
  return id;
}

std::vector<std::vector<XodrLaneId>> Roadgraph::find_all_paths_in_subgraph(
    const std::vector<XodrLaneEdgeType> &edge_type_subset,
    const std::vector<XodrLaneId> &lane_id_subset) {
  struct Predicate {
    bool operator()(XodrLaneGraph::edge_descriptor edge_des) const {
      XodrLaneEdgeType lane_edge_type = (*g)[edge_des].get_edge_type();
      return (find(edge_type_subset.begin(), edge_type_subset.end(),
                   lane_edge_type) != edge_type_subset.end());
    }
    bool operator()(XodrLaneGraph::vertex_descriptor vertex_des) const {
      XodrLaneId lane_id = (*g)[vertex_des].global_lane_id;
      return (find(lane_id_subset.begin(), lane_id_subset.end(), lane_id) !=
              lane_id_subset.end());
    }

    const XodrLaneGraph *g;
    std::vector<XodrLaneEdgeType> edge_type_subset;
    std::vector<XodrLaneId> lane_id_subset;
  } predicate{&g_, edge_type_subset, lane_id_subset};
  boost::filtered_graph<XodrLaneGraph, Predicate, Predicate> filtered_graph(
      g_, predicate, predicate);

  std::vector<std::vector<XodrLaneId>> paths;

  for (vertex_t const &start_vertex :
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
    for (auto const &goal_vertex :
         boost::make_iterator_range(vertices(filtered_graph))) {
      if (out_degree(goal_vertex, filtered_graph) > 0) {
        continue;
      }
      // goal_vertex has no successors
      std::vector<XodrLaneId> path;
      bool reachable = true;
      boost::graph_traits<XodrLaneGraph>::vertex_descriptor current = goal_vertex;
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

XodrLanePtr Roadgraph::get_laneptr(const XodrLaneId &id) const {
  std::pair<vertex_t, bool> lane_vertex_pair = get_vertex_by_lane_id(id);
  if (lane_vertex_pair.second) {
    return g_[lane_vertex_pair.first].lane;
  } else {
    return NULL;
  }
}

std::vector<XodrLaneId> Roadgraph::get_all_laneids() const {
  std::vector<XodrLaneId> ids;
  std::vector<vertex_t> vertices = get_vertices();
  for (auto const &v : vertices) {
    if (get_lane_graph()[v].lane->get_lane_position() != 0) {
      ids.push_back(get_lane_graph()[v].global_lane_id);
    }
  }
  return ids;
}

std::pair<XodrLaneId, bool> Roadgraph::getLanePlanView(const XodrLaneId lane_id) const {
  XodrLanePtr lane = get_laneptr(lane_id);
  if (lane->get_lane_position() == 0) {
      return std::make_pair(lane_id, true);
  }
  else {
    auto inner_lane = get_inner_neighbor(lane->get_id());
    if (inner_lane.second) {
      return getLanePlanView(inner_lane.first);
    }
    else {
      // their appears to be some error
      return std::make_pair(0, false);
    }
  }
}

std::pair<XodrLaneId, bool> Roadgraph::get_inner_neighbor(
    const XodrLaneId &lane_id) const {
  std::vector<std::pair<XodrLaneId, bool>> neighbors =
      get_neighbor_from_edgetype(lane_id, INNER_NEIGHBOR_EDGE);
  std::pair<XodrLaneId, bool> neighbor =
      neighbors.front();  // inner neighbor is unique
  return neighbor;
}

std::pair<XodrLaneId, bool> Roadgraph::get_outer_neighbor(
    const XodrLaneId &lane_id) const {
  std::vector<std::pair<XodrLaneId, bool>> tmp =
      get_neighbor_from_edgetype(lane_id, OUTER_NEIGHBOR_EDGE);
  return tmp.front();  // for a non planview edge the outer neighbor is unique
}

std::pair<XodrLaneId, bool> Roadgraph::get_outer_neighbor_but_not(
    const XodrLaneId &lane_id, const XodrLaneId &but_not) {
  std::vector<std::pair<XodrLaneId, bool>> tmp =
      get_neighbor_from_edgetype(lane_id, OUTER_NEIGHBOR_EDGE);
  for (auto &t : tmp) {
    if (t.first != but_not) {
      return t;
    }
  }
  return std::make_pair<XodrLaneId, bool>(0, false);  // error case
}

std::vector<XodrLaneId> Roadgraph::get_all_neighbors(const XodrLaneId &lane_id) const {
  XodrLanePtr lane = get_laneptr(lane_id);
  if (lane->get_lane_position() == 0) {
    throw std::runtime_error("get_all_neighbors was called with the plan view");
  }

  std::vector<XodrLaneId> neighbors;

  std::pair<XodrLaneId, bool> current_neighbor = get_inner_neighbor(lane_id);
  std::pair<XodrLaneId, bool> next_neighbor = std::make_pair(0, false);

  // Inner neighbors
  if (current_neighbor.second) {
    // We do not want to add the planview to the vector of neighbors, therefore
    // check if the current neighbor has an inner neighbor
    next_neighbor = get_inner_neighbor(current_neighbor.first);
  }
  while (next_neighbor.second) {
    neighbors.push_back(current_neighbor.first);
    current_neighbor = next_neighbor;
    next_neighbor = get_inner_neighbor(current_neighbor.first);
  }

  // Outer neighbors
  current_neighbor = get_outer_neighbor(lane_id);
  while (current_neighbor.second) {
    neighbors.push_back(current_neighbor.first);
    current_neighbor = get_outer_neighbor(current_neighbor.first);
  }

  return neighbors;
}

bool Roadgraph::has_lane(const XodrLaneId &lane_id) const {
  boost::graph_traits<XodrLaneGraph>::vertex_iterator i, end;
  for (boost::tie(i, end) = boost::vertices(g_); i != end; ++i) {
    if (g_[*i].global_lane_id == lane_id) {
      return true;
    }
  }
  return false;
}

void Roadgraph::print_graph(const char *filename) {
  std::ofstream dotfile(filename);
  print_graph(dotfile);
  dotfile.close();
}

void Roadgraph::print_graph(std::ofstream &dotfile) {
  write_graphviz(dotfile, g_,
                 make_vertex_label_writer_graph(
                     boost::get(&XodrLaneVertex::road_id, g_),
                     boost::get(&XodrLaneVertex::global_lane_id, g_),
                     boost::get(&XodrLaneVertex::lane, g_)),
                 make_edge_label_writer_text(get(&XodrLaneEdge::edge_type, g_)));
}

std::vector<vertex_t> Roadgraph::get_vertices() const {
  std::vector<vertex_t> vertex_descriptors;
  boost::graph_traits<XodrLaneGraph>::vertex_iterator i, end;
  for (boost::tie(i, end) = boost::vertices(g_); i != end; ++i) {
    vertex_descriptors.push_back(*i);
  }
  return vertex_descriptors;
}

std::vector<vertex_t> Roadgraph::get_next_vertices(
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

std::vector<edge_t> Roadgraph::get_out_edges(vertex_t current_vertex) const {
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

std::vector<edge_t> Roadgraph::get_edges() const {
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

edge_t Roadgraph::get_edge_descr(vertex_t from, vertex_t to) const {
  return boost::edge(from, to, g_).first;
}

std::pair<vertex_t, bool> Roadgraph::get_vertex_by_lane_id(
    const XodrLaneId &lane_id) const {
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
  for (auto const &road_element : map->get_roads()) {  // std::map
    for (auto const &lane_section_element :
         road_element.second->get_lane_sections()) {  // std::vector
      for (auto const &lane_element :
           lane_section_element->get_lanes()) {  // std::map
        add_lane(road_element.first, lane_element.second);
      }
    }
  }
}

void Roadgraph::GeneratePreAndSuccessors(OpenDriveMapPtr map) {
  // add successors, predecessors
  for (auto const &road_element : map->get_roads()) {  // std::map
    XodrRoadId successor_id = road_element.second->get_link()
                              .get_successor()
                              .id_;  // this is the position!!!!!! (-4, 4)
    XodrRoadId predecessor_id = road_element.second->get_link()
                                .get_predecessor()
                                .id_;  // this is the position!!!!!! (-4, 4)
    if (successor_id > 1000) {
      continue;
    }

    // make sure that there is a predecessor!!
    XodrLaneSectionPtr successor_lane_section = nullptr;
    XodrLaneSectionPtr predecessor_lane_section = nullptr;

    // TODO(@hart): That's pretty ugly, move check for road_id to map

    if (road_element.second->get_link().get_successor().type_ ==
        "road") {  // successor_id == iter->first
      auto it = map->get_roads().find(successor_id);
      if (it != map->get_roads().end()) {
        XodrRoadPtr successor_road = map->get_roads().at(successor_id);
        successor_lane_section = successor_road->get_lane_sections().front();
      }
    } else {
      LOG(INFO) << "XodrRoad has no successor road. \n";
    }

    if (road_element.second->get_link().get_predecessor().type_ ==
        "road") {  // predecessor_id == iter->first
      auto it = map->get_roads().find(predecessor_id);
      if (it != map->get_roads().end()) {
        XodrRoadPtr predecessor_road = map->get_roads().at(predecessor_id);
        predecessor_lane_section = predecessor_road->get_lane_sections().back();
      }
    } else {
      LOG(INFO) << "XodrRoad has no predeseccor road. \n";
    }

    // TODO(@hart): there could be mult. lane_sections
    for (auto const &lane_section_element :
         road_element.second->get_lane_sections()) {  // std::vector
      for (auto const &lane_element :
           lane_section_element->get_lanes()) {  // std::map
        // add successor edge
        if (successor_lane_section) {
          XodrLanePosition successor_lane_position =
              lane_element.second->get_link().to_position;

          XodrLanePtr successor_lane = successor_lane_section->get_lane_by_position(
                successor_lane_position);
          if (successor_lane_position == 0) {
            auto lane_plan_view = getLanePlanView(lane_element.first);
            add_road_successor(lane_plan_view.second, successor_lane->get_id());
          } else {
            if (successor_lane) {
              bool success =
                  add_lane_successor(lane_element.first, successor_lane->get_id());
            }
          }
        }

        // does not always have predecessor
        try {
          // search for predecessor_lane_position in previos lane section
          if (predecessor_lane_section) {
            XodrLanePosition predecessor_lane_position =
                lane_element.second->get_link().from_position;
            XodrLanePtr predecessor_lane = predecessor_lane_section->get_lane_by_position(predecessor_lane_position);
            if (predecessor_lane_position == 0) {
              auto lane_plan_view = getLanePlanView(lane_element.first);
              add_road_successor(lane_plan_view.second, predecessor_lane->get_id());
            }
            else {
              // if found add; convert predecessor to successor
              if (predecessor_lane) {
                bool success =
                    add_lane_successor(predecessor_lane->get_id(), lane_element.first);
              }
            }
          }
        } catch (const std::exception &ex) {
          LOG(INFO) << "XodrRoad has no predeseccor road. \n";
        }
      }
    }
  }
}

void Roadgraph::GenerateNeighbours(OpenDriveMapPtr map) {
  // add neighbor edges
  for (auto const &road_element : map->get_roads()) {  // std::map
    for (auto const &lane_section_element :
         road_element.second->get_lane_sections()) {  // std::vector
      for (auto const &lane_element :
           lane_section_element->get_lanes()) {  // std::map
        if (lane_element.second->get_lane_position() != 0) {
          XodrLanePosition inner_lane_pos;
          if (lane_element.second->get_lane_position() > 0) {
            inner_lane_pos = lane_element.second->get_lane_position() - 1;
          } else {
            inner_lane_pos = lane_element.second->get_lane_position() + 1;
          }

          XodrLanePtr inner_lane =
              lane_section_element->get_lane_by_position(inner_lane_pos);
          if (inner_lane) {
            bool success_inner = add_inner_neighbor(
                inner_lane->get_id(), lane_element.second->get_id());
            bool success_outer = add_outer_neighbor(
                inner_lane->get_id(), lane_element.second->get_id());
          }
        }
      }
    }
  }
}

void Roadgraph::GenerateFromJunctions(OpenDriveMapPtr map) {
  // map junctions
  for (auto const &road_element : map->get_junctions()) {  // std::map
    for (auto const &connection_element :
         road_element.second->get_connections()) {  // std::map
      XodrRoadPtr incoming_road =
          map->get_road(connection_element.second.incoming_road_);
      XodrRoadPtr connecting_road =
          map->get_road(connection_element.second.connecting_road_);
      XodrLaneSectionPtr pre_lane_section =
          incoming_road->get_lane_sections().front();
      XodrLaneSectionPtr successor_lane_section =
          connecting_road->get_lane_sections().front();
      for (auto const &lane_link_element :
           connection_element.second.get_lane_links()) {
        // add successor edge
        if (pre_lane_section && successor_lane_section) {
          try {
            XodrLanePtr pre_lane = pre_lane_section->get_lane_by_position(
                lane_link_element.from_position);
            XodrLanePtr successor_lane =
                successor_lane_section->get_lane_by_position(
                    lane_link_element.to_position);
            if (pre_lane && successor_lane) {
              bool success =
                  add_lane_successor(pre_lane->get_id(), successor_lane->get_id());
              // also connect road elements (through plan view)
              auto pre_plan_view = getLanePlanView(pre_lane->get_id());
              auto succ_plan_view = getLanePlanView(pre_lane->get_id());
              success = add_road_successor(pre_plan_view.first, succ_plan_view.first);
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
  std::vector<vertex_t> vertices = get_vertices();
  for (auto const &v : vertices) {
    // g_[v].polygon = ComputeXodrLanePolygon(get_lane_graph()[v].lane->get_id());
    if (get_lane_graph()[v].lane->get_lane_position() != 0) {
      auto p = ComputeXodrLanePolygon(get_lane_graph()[v].lane->get_id());
      if (p.second) {
        g_[v].polygon = p.first;
      }
    }
    // auto l = get_lane_graph()[v].lane->get_id();
  }
}

void Roadgraph::Generate(OpenDriveMapPtr map) {
  GenerateVertices(map);

  GeneratePreAndSuccessors(map);

  GenerateNeighbours(map);

  GenerateFromJunctions(map);

  GeneratePolygonsForVertices();
}

std::pair<XodrLanePtr, XodrLanePtr> Roadgraph::ComputeXodrLaneBoundaries(
    const XodrLaneId &lane_id) const {
  XodrLanePtr inner, outer;
  std::pair<vertex_t, bool> v = get_vertex_by_lane_id(lane_id);
  auto l = get_lane_graph()[v.first].lane;
  // assert(l->get_lane_position() != 0); // make sure we are not at the
  // planview, as a driving corridor cannot be computed from here.
  outer = l;

  std::pair<XodrLaneId, bool> innerid = get_inner_neighbor(lane_id);
  if (innerid.second) {
    std::pair<vertex_t, bool> v_inner = get_vertex_by_lane_id(innerid.first);
    if (v_inner.second) {
      inner = get_lane_graph()[v_inner.first].lane;
    } else {
      inner = NULL;
    }
  } else {  // you are probably at the planview and do not have inner lanes?
  }
  return std::make_pair(inner, outer);
}

std::pair<std::vector<XodrLanePtr>, std::vector<XodrLanePtr>>
Roadgraph::ComputeRouteBoundaries(const std::vector<XodrLaneId> &horizon) const {
  std::vector<XodrLanePtr> inner, outer;
  if (!horizon.empty()) {
    for (auto &h : horizon) {
      auto lane_boundaries = ComputeXodrLaneBoundaries(h);
      inner.push_back(lane_boundaries.first);
      outer.push_back(lane_boundaries.second);
    }
  }
  return std::make_pair(inner, outer);
}

std::pair<PolygonPtr, bool> Roadgraph::ComputeXodrLanePolygon(
    const XodrLaneId &lane_id) const {
  bool success = false;
  std::pair<XodrLanePtr, XodrLanePtr> lb = ComputeXodrLaneBoundaries(lane_id);

  PolygonPtr polygon = std::make_shared<modules::geometry::Polygon>();
  // inner
  if (lb.first && lb.second) {
    success = true;

    for (auto const &p : lb.first->get_line()) {
      polygon->add_point(p);
    }
    // outer
    auto reversed_outer = lb.second->get_line();
    reversed_outer.reverse();

    for (auto const &p : reversed_outer) {
      polygon->add_point(p);
    }
    // Polygons need to be closed!
    polygon->add_point(*(lb.first->get_line().begin()));
  }
  return std::make_pair(polygon, success);
}

PolygonPtr Roadgraph::get_lane_polygon_by_id(const XodrLaneId &lane_id) {
  auto v = get_vertex_by_lane_id(lane_id);
  return get_lane_graph().operator[](v.first).polygon;
}

XodrRoadId Roadgraph::get_road_by_lane_id(const XodrLaneId &lane_id) {
  auto v = get_vertex_by_lane_id(lane_id);
  return get_lane_graph().operator[](v.first).road_id;
}

}  // namespace map
}  // namespace world
}  // namespace modules
