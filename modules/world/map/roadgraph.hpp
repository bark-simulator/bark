// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_WORLD_ROADGRAPH_HPP_
#define MODULES_WORLD_ROADGRAPH_HPP_

#include <vector>
#include <string>
#include <ostream>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/topological_sort.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/filtered_graph.hpp>
#include "modules/world/opendrive/opendrive.hpp"
#include "modules/geometry/polygon.hpp"

namespace modules {
namespace world {
namespace map {

using namespace modules::world::opendrive;
using namespace boost;

typedef std::shared_ptr<modules::geometry::Polygon> PolygonPtr;

struct LaneVertex {
  RoadId road_id;
  LaneId global_lane_id;
  LaneId get_global_line_id() { return global_lane_id; }
  LanePtr get_lane() { return lane; }
  LanePtr lane;
  PolygonPtr polygon;
  LaneVertex() : road_id(0), global_lane_id(0), lane(NULL), polygon(NULL) {}
  LaneVertex(int road_id_in, int global_lane_id_in, LanePtr lane_in) : road_id(road_id_in), global_lane_id(global_lane_id_in), lane(lane_in), polygon(NULL) {}
};

enum LaneEdgeType {
  SUCCESSOR_EDGE = 0,
  INNER_NEIGHBOR_EDGE = 1,
  OUTER_NEIGHBOR_EDGE = 2
};

struct LaneEdge {
  LaneEdgeType edge_type;
  float weight; //! @todo tobias: for shortest path calculation: a very basic implementation!
  LaneEdgeType get_edge_type() const { return edge_type; }
  LaneEdge() : edge_type(SUCCESSOR_EDGE), weight(1) {}
  LaneEdge(LaneEdgeType edge_type_in) : edge_type(edge_type_in), weight(edge_type_in==SUCCESSOR_EDGE?1:10) {}
};

typedef boost::adjacency_list<vecS, vecS, bidirectionalS, LaneVertex, LaneEdge> LaneGraph;
typedef boost::graph_traits<LaneGraph>::vertex_descriptor vertex_t;
typedef boost::graph_traits<LaneGraph>::edge_descriptor edge_t;

struct LaneTypeDrivingAndEdgeTypeSuccessorPredicate { // both edge and vertex
  bool operator()(LaneGraph::edge_descriptor ed) const      { 
    bool filtered_s = (*g)[boost::source(ed, *g)].lane->get_lane_type()==LaneType::DRIVING;
    bool filtered_t = (*g)[boost::target(ed, *g)].lane->get_lane_type()==LaneType::DRIVING;
    bool filtered_e = (*g)[ed].edge_type==LaneEdgeType::SUCCESSOR_EDGE;
        
    bool filtered = filtered_s && filtered_t && filtered_e;
    return filtered; 
  } 
      
  bool operator()(LaneGraph::vertex_descriptor vd) const {
    bool filtered = (*g)[vd].lane->get_lane_type()==LaneType::DRIVING; 
    return filtered;
  }
  LaneGraph* g;
};

typedef boost::filtered_graph<LaneGraph, LaneTypeDrivingAndEdgeTypeSuccessorPredicate, LaneTypeDrivingAndEdgeTypeSuccessorPredicate> FilteredLaneGraph;

class Roadgraph {
 public:
  Roadgraph() {}

  // TODO: Move to LaneGraph, but LaneGraph is not wrapped to python
  PolygonPtr get_lane_polygon_by_id(const LaneId &lane_id);

  LaneId add_lane(const RoadId& road_id, const LanePtr& laneptr);

  bool add_inner_neighbor(const LaneId& inner_id, const LaneId& outer_id);

  bool add_outer_neighbor(const LaneId& inner_id, const LaneId& outer_id);

  bool add_successor(const LaneId& prev, const LaneId& succ);

  std::vector<LaneId> get_successor_lanes(const LaneId& lane_id) const;

  std::vector<LaneId> get_predecessor_lanes(const LaneId& lane_id) const;

  bool check_id_in_filtered_graph(const FilteredLaneGraph& fg, const LaneId& lane_id) const;

  std::vector<LaneId> find_path(const LaneId& startid, const LaneId& goalid);

  std::vector<std::vector<LaneId>> find_all_paths_in_subgraph(const std::vector<LaneEdgeType> &edge_type_subset, const std::vector<LaneId> &lane_id_subset);

  LanePtr get_laneptr(const LaneId& id) const;

  std::vector<LaneId> get_all_laneids() const;

  //! LaneId of the neighboring lane and a flag if it exists or not
  std::pair<LaneId, bool> get_inner_neighbor(const LaneId& lane_id) const;

  //! LaneId of the neighboring lane and a flag if it exists or not
  //! @note make sure to not call this function on the planview lane!
  std::pair<LaneId, bool> get_outer_neighbor(const LaneId& lane_id) const;

  //! LaneId of the neighboring lane and a flag if it exists or not, usecase: lane_id is the planview
  //! @param lane_id queried lane id
  //! @param from the query answer return the lane id that is not but_not
  std::pair<LaneId, bool> get_outer_neighbor_but_not(const LaneId& lane_id, const LaneId& but_not);

  //! LaneIds of all neighboring lanes in the same driving direction. Includes neighbors of neighbors
  //! @note cannot be called with the planview lane!
  std::vector<LaneId> get_all_neighbors(const LaneId &lane_id) const;

  bool has_lane(const LaneId& lane_id) const;

  void print_graph(const char* filename);

  void print_graph(std::ofstream& dotfile);

  LaneGraph get_lane_graph() const {
    return g_;
  }

  LaneVertex get_vertex(vertex_t v_des) const {
    return g_[v_des];
  }

  std::vector<vertex_t> get_vertices() const;

  LaneEdge get_edge(edge_t e_des) const {
    return g_[e_des];
  }

  std::vector<vertex_t> get_next_vertices(vertex_t current_vertex) const;

  std::vector<edge_t> get_out_edges(vertex_t current_vertex) const;

  std::vector<edge_t> get_edges() const;

  edge_t get_edge_descr(vertex_t from, vertex_t to) const;

  std::pair<vertex_t,bool> get_vertex_by_lane_id(const LaneId& lane_id) const;

  std::pair< LanePtr, LanePtr > ComputeLaneBoundaries(const LaneId& lane_id) const;

  // TODO Klemens: change to LanePtr to Line
  std::pair< std::vector<LanePtr>, std::vector<LanePtr> > ComputeRouteBoundaries(const std::vector<LaneId>& horizon) const;

  std::pair<PolygonPtr, bool> ComputeLanePolygon(const LaneId& lane_id) const;

  void GenerateVertices(OpenDriveMapPtr map);

  void GeneratePreAndSuccessors(OpenDriveMapPtr map);

  void GenerateNeighbours(OpenDriveMapPtr map);

  void GenerateFromJunctions(OpenDriveMapPtr map);

  void GeneratePolygonsForVertices();

  void Generate(OpenDriveMapPtr map);

  RoadId get_road_by_lane_id(const LaneId &lane_id);

 private:
  LaneGraph g_;

  bool add_edge_of_type(const LaneId& source_id, const LaneId& target_id, const LaneEdgeType& edgetype) {
    LaneEdge edge = LaneEdge(edgetype);
    std::pair<vertex_t,bool> source_lane = get_vertex_by_lane_id(source_id);
    std::pair<vertex_t,bool> target_lane = get_vertex_by_lane_id(target_id);
    if(source_lane.second && target_lane.second) {
      boost::add_edge(source_lane.first, target_lane.first, edge, g_);
      return true;
    } else {
      return false;
    }
  }

  std::vector<std::pair<LaneId, bool> > get_neighbor_from_edgetype(const LaneId& lane_id, const LaneEdgeType edge_type) const { //! we can have two outer neighbors
    std::vector<std::pair<LaneId, bool> > retval;
    std::pair<vertex_t, bool> lane_vertex_pair = get_vertex_by_lane_id(lane_id);
    boost::graph_traits<LaneGraph>::out_edge_iterator i, end;
    for (boost::tie(i, end) = boost::out_edges(lane_vertex_pair.first, g_); i != end; ++i) {
      if(g_[*i].edge_type == edge_type) {
        vertex_t target = boost::target(*i,g_);
        retval.push_back(std::make_pair(g_[target].global_lane_id, true));
      }
    }
    if (retval.empty()) {
      retval.push_back(std::make_pair(0, false));
    }
    return retval;
  }

  template <class RoadIdMap, class LaneIdMap, class LaneMap>
  class my_vertex_writer_graph {
  public:
    my_vertex_writer_graph(RoadIdMap r, LaneIdMap l, LaneMap lanemap) : rm(r), lm(l), lanemap_(lanemap) {}
    template <class LaneVertex>
    void operator()(std::ostream &out, const LaneVertex& s) const {
      out << "[" << 
        "label=\"" << "road_id=" << rm[s] << " lane_id=" << lm[s] << " lane_pos=" << lanemap_[s]->get_lane_position() << 
        "\"]";
    }
  private:
    RoadIdMap rm;
    LaneIdMap lm;
    LaneMap lanemap_;
  };

  template <class RoadIdMap, class LaneIdMap, class LaneMap>
  inline my_vertex_writer_graph<RoadIdMap, LaneIdMap, LaneMap>
  make_vertex_label_writer_graph(RoadIdMap r, LaneIdMap l, LaneMap lm) {
    return my_vertex_writer_graph<RoadIdMap, LaneIdMap, LaneMap> (r, l, lm);
  }

  template <class TypeMap>
  class my_edge_writer_text {
  public:
    my_edge_writer_text(TypeMap t) : tm(t) {}
    template <class LaneEdge>
    void operator()(std::ostream &out, const LaneEdge& e) const {
      const char* color = tm[e]==0?"red":"blue";
      out << "[label=\"" << tm[e] << "\"" <<  "color=\"" << color << "\"" << "]";
    }
  private:
    TypeMap tm;
  };

  template <class TypeMap>
  inline my_edge_writer_text<TypeMap>
  make_edge_label_writer_text(TypeMap t) {
    return my_edge_writer_text<TypeMap> (t);
  }

};

typedef std::shared_ptr<Roadgraph> RoadgraphPtr;

}  // namespace map
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_ROADGRAPH_HPP_
