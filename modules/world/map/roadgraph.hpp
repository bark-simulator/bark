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
#include "modules/world/opendrive/opendrive.hpp"

namespace modules {
namespace world {
namespace map {

using namespace modules::world::opendrive;
using namespace boost;


struct LaneVertex {
  RoadId road_id;
  LaneId global_lane_id;
  LaneId get_global_line_id() { return global_lane_id; }
  LanePtr get_lane() { return lane; }
  LanePtr lane;
  LaneVertex() : road_id(0), global_lane_id(0), lane(NULL) {}
  LaneVertex(int road_id_in, int global_lane_id_in, LanePtr lane_in) : road_id(road_id_in), global_lane_id(global_lane_id_in), lane(lane_in) {}
};

enum LaneEdgeType {
  SUCCESSOR_EDGE = 0,
  INNER_NEIGHBOR_EDGE = 1,
  OUTER_NEIGHBOR_EDGE = 2
};

struct LaneEdge {
  LaneEdgeType edge_type;
  float weight; //! @todo tobias: for shortest path calculation: a very basic implementation!
  LaneEdgeType get_edge_type() { return edge_type; }
  LaneEdge() : edge_type(SUCCESSOR_EDGE), weight(1) {}
  LaneEdge(LaneEdgeType edge_type_in) : edge_type(edge_type_in), weight(edge_type_in==SUCCESSOR_EDGE?1:10) {}
};


typedef boost::adjacency_list<vecS, vecS, directedS, LaneVertex, LaneEdge> LaneGraph;
typedef boost::graph_traits<LaneGraph>::vertex_descriptor vertex_t;
typedef boost::graph_traits<LaneGraph>::edge_descriptor edge_t;

class Roadgraph {
 public:
  Roadgraph() {}

  LaneId add_lane(const RoadId& road_id, const LanePtr& laneptr) {
    LaneVertex lane = LaneVertex(road_id, laneptr->get_id(), laneptr);
    boost::add_vertex(lane, g_);
    return laneptr->get_id();
  }

  bool add_inner_neighbor(const LaneId& inner_id, const LaneId& outer_id) {
    return add_edge_of_type( outer_id, inner_id, INNER_NEIGHBOR_EDGE);
  }

  bool add_outer_neighbor(const LaneId& inner_id, const LaneId& outer_id) {
    return add_edge_of_type( inner_id, outer_id, OUTER_NEIGHBOR_EDGE);
  }

  bool add_successor(const LaneId& prev, const LaneId& succ) {
    return add_edge_of_type( prev, succ, SUCCESSOR_EDGE);
  }

  std::vector<LaneId> get_successor_lanes(const LaneId& lane_id)
  {
    std::pair<vertex_t,bool> lane_vertex_pair = get_vertex_by_lane_id(lane_id);
    boost::graph_traits<LaneGraph>::out_edge_iterator i, end;
    std::vector<LaneId> successor_lanes;
    for (boost::tie(i, end) = boost::out_edges(lane_vertex_pair.first, g_); i != end; ++i) {
      if(g_[*i].edge_type == SUCCESSOR_EDGE) {
        vertex_t target = boost::target(*i,g_);
        successor_lanes.push_back(g_[target].global_lane_id);
      }
    }
    return successor_lanes;
  }

  std::vector<LaneId> find_path(const LaneId& startid, const LaneId& goalid) {
    std::vector<LaneId> path;

    std::pair<vertex_t, bool> start_vertex = get_vertex_by_lane_id(startid);
    std::pair<vertex_t, bool> goal_vertex = get_vertex_by_lane_id(goalid);

    if (start_vertex.second && goal_vertex.second)
    {
      std::vector<vertex_t> p(boost::num_vertices(g_));
      std::vector<int> d(boost::num_vertices(g_));
      boost::property_map<LaneGraph, float LaneEdge::*>::type weightmap = boost::get(&LaneEdge::weight, g_);

      boost::dijkstra_shortest_paths(g_, start_vertex.first,
                                     predecessor_map(boost::make_iterator_property_map(p.begin(), get(boost::vertex_index, g_)))
                                         .distance_map(boost::make_iterator_property_map(d.begin(), get(boost::vertex_index, g_)))
                                         .weight_map(weightmap));

      // get shortest path from predecessor map
      int stop_the_loop = boost::num_vertices(g_);
      int idx = 0;
      boost::graph_traits< LaneGraph >::vertex_descriptor current = goal_vertex.first;
      while(current!=start_vertex.first && idx < stop_the_loop) {
        path.push_back(g_[current].global_lane_id);
        current=p[current];
        ++idx;
      }
      path.push_back(g_[start_vertex.first].global_lane_id);
      std::reverse(path.begin(), path.end());

      //for (auto &p : path) {
      //    std::cout << p << " ";
      //}
      //std::cout << std::endl;
    }

    return path;
  }

  //std::vector<LaneId> find_path(const LaneId& start, const RoadId& goal)
  //{
  //  std::vector<LaneId> path;
  //  return path;
  //}

  LanePtr get_laneptr(const LaneId& id){
    std::pair<vertex_t,bool> lane_vertex_pair = get_vertex_by_lane_id(id);
    if(lane_vertex_pair.second) {
      return g_[lane_vertex_pair.first].lane;
    } else {
      return NULL;
    }
  }

  //! LaneId of the neighboring lane and a flag if it exists or not
  std::pair<LaneId, bool> get_inner_neighbor(const LaneId& lane_id) {
    std::vector<std::pair<LaneId, bool> > neighbors =
     get_neighbor_from_edgetype(lane_id, INNER_NEIGHBOR_EDGE); 
    std::pair<LaneId, bool> neighbor = neighbors.front(); //inner neighbor is unique
    // handle special case: one lane next to the planview, move to the next outer lane
    if(neighbor.second) { //neighboring lane exists
      LanePtr lp = get_laneptr(neighbor.first);
      if(lp->get_lane_position() == 99999) { //lane pos 0 is the planview: neighbor is the planview //TODO 999 is a quickfix
        std::vector<std::pair<LaneId, bool> > next_neighbors = 
          get_neighbor_from_edgetype(neighbor.first, OUTER_NEIGHBOR_EDGE);
        for(auto nn : next_neighbors) { //find the non-queried lane id
          if(nn.first != lane_id) {
            return nn;
          }
        }
      } 
    } 
    return neighbor;
  }

  //! LaneId of the neighboring lane and a flag if it exists or not
  std::pair<LaneId, bool> get_outer_neighbor(const LaneId& lane_id) {
    std::vector<std::pair<LaneId, bool> > tmp = get_neighbor_from_edgetype(lane_id, OUTER_NEIGHBOR_EDGE);
    return tmp.front(); // for a non planview edge the outer neighbor is unique
  }

  bool has_lane(const LaneId& lane_id){
    boost::graph_traits<LaneGraph>::vertex_iterator i, end;
    for (boost::tie(i, end) = boost::vertices(g_); i != end; ++i) {
      if (g_[*i].global_lane_id == lane_id) {
        return true;
      }
    }
    return false;
  }

  void print_graph(const char* filename) {
    std::ofstream dotfile(filename);
    print_graph(dotfile);
    dotfile.close();
  }

  void print_graph(std::ofstream& dotfile){
    	write_graphviz(dotfile, g_
		, make_vertex_label_writer_graph(boost::get(&LaneVertex::road_id, g_), boost::get(&LaneVertex::global_lane_id, g_), boost::get(&LaneVertex::lane, g_))
		, make_edge_label_writer_text(get(&LaneEdge::edge_type, g_))
    );
  }

  LaneGraph get_lane_graph() const {
    return g_;
  }

  LaneVertex get_vertex(vertex_t v_des) {
    return g_[v_des];
  }

  std::vector<vertex_t> get_vertices() {
    std::vector<vertex_t> vertex_descriptors;
    boost::graph_traits<LaneGraph>::vertex_iterator i, end;
    for (boost::tie(i, end) = boost::vertices(g_); i != end; ++i) {
      vertex_descriptors.push_back(*i);
    }
    return vertex_descriptors;
  }

  LaneEdge get_edge(edge_t e_des) {
    return g_[e_des];
  }

  std::vector<vertex_t> get_next_vertices(vertex_t current_vertex) {
    std::vector<vertex_t> vertex_descriptors;
    boost::graph_traits<LaneGraph>::out_edge_iterator ei, ei_end;
    for (boost::tie(ei, ei_end) = boost::out_edges(current_vertex, g_); ei != ei_end; ei++) {
      vertex_t target = boost::target(*ei, g_);
      vertex_descriptors.push_back(target);
    }
    return vertex_descriptors;
  }

  std::vector<edge_t> get_out_edges(vertex_t current_vertex) {
    std::vector<edge_t> edge_descriptors;
    boost::graph_traits<LaneGraph>::out_edge_iterator ei, ei_end;
    for (boost::tie(ei, ei_end) = boost::out_edges(current_vertex, g_); ei != ei_end; ei++) {
      vertex_t source = boost::source(*ei, g_);
      vertex_t target = boost::target(*ei, g_);
      auto edge = boost::edge(source, target, g_);
      edge_descriptors.push_back(edge.first);
    }
    return edge_descriptors;
  }

  std::vector<edge_t> get_edges() {
    std::vector<edge_t> edge_descriptors;
    boost::graph_traits<LaneGraph>::edge_iterator ei, ei_end;
    for (boost::tie(ei, ei_end) = boost::edges(g_); ei != ei_end; ei++) {
      vertex_t source = boost::source(*ei, g_);
      vertex_t target = boost::target(*ei, g_);
      auto edge = boost::edge(source, target, g_);
      edge_descriptors.push_back(edge.first);
    }
    return edge_descriptors;
  }

  edge_t get_edge_descr(vertex_t from, vertex_t to) {
    return boost::edge(from, to, g_).first;
  }

  std::pair<vertex_t,bool> get_vertex_by_lane_id(const LaneId& lane_id) {
    std::pair<vertex_t, bool> retval;
    retval.second = false;
    boost::graph_traits<LaneGraph>::vertex_iterator i, end;
    for (boost::tie(i, end) = boost::vertices(g_); i != end; ++i) {
      if (g_[*i].global_lane_id == lane_id) {
        retval.first = *i;
        retval.second = true;
        break;
      }
    }
    return retval;
  }

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

  std::vector<std::pair<LaneId, bool> > get_neighbor_from_edgetype(const LaneId& lane_id, const LaneEdgeType edge_type) { //! we can have two outer neighbors
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
