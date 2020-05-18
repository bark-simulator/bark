// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_ROADGRAPH_HPP_
#define MODULES_WORLD_ROADGRAPH_HPP_

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/topological_sort.hpp>
#include <ostream>
#include <string>
#include <utility>
#include <vector>
#include "modules/geometry/polygon.hpp"
#include "modules/world/opendrive/opendrive.hpp"

namespace modules {
namespace world {
namespace map {

using namespace modules::world::opendrive;
using namespace boost;

typedef std::shared_ptr<modules::geometry::Polygon> PolygonPtr;

struct XodrLaneVertex {
  XodrRoadId road_id;
  XodrLaneId global_lane_id;
  XodrLaneId GetGlobalLineId() { return global_lane_id; }
  XodrLanePtr GetLane() { return lane; }
  XodrLanePtr lane;
  PolygonPtr polygon;
  XodrLaneVertex() : road_id(0), global_lane_id(0), lane(NULL), polygon(NULL) {}
  XodrLaneVertex(int road_id_in, int global_lane_id_in, XodrLanePtr lane_in)
      : road_id(road_id_in),
        global_lane_id(global_lane_id_in),
        lane(lane_in),
        polygon(NULL) {}
};

enum XodrLaneEdgeType {
  LANE_SUCCESSOR_EDGE = 0,
  INNER_NEIGHBOR_EDGE = 1,
  OUTER_NEIGHBOR_EDGE = 2,
  ROAD_SUCCESSOR_EDGE = 3
};

struct XodrLaneEdge {
  XodrLaneEdgeType edge_type;
  float weight;  //! @todo tobias: for shortest path calculation: a very basic
                 //! implementation!
  XodrLaneEdgeType GetEdgeType() const { return edge_type; }
  XodrLaneEdge() : edge_type(LANE_SUCCESSOR_EDGE), weight(1) {}
  XodrLaneEdge(XodrLaneEdgeType edge_type_in) :
    edge_type(edge_type_in),
    weight(edge_type_in == LANE_SUCCESSOR_EDGE ? 1 : 10) {}
};

typedef boost::adjacency_list<vecS, vecS, bidirectionalS, XodrLaneVertex,
                              XodrLaneEdge>
    XodrLaneGraph;
typedef boost::graph_traits<XodrLaneGraph>::vertex_descriptor vertex_t;
typedef boost::graph_traits<XodrLaneGraph>::edge_descriptor edge_t;

struct TypeDrivingAndEdgeTypeLaneSuccessor {  // both edge and vertex
  bool operator()(XodrLaneGraph::edge_descriptor ed) const {
    bool filtered_s = (*g)[boost::source(ed, *g)].lane->GetLaneType() ==
                      XodrLaneType::DRIVING;
    bool filtered_t = (*g)[boost::target(ed, *g)].lane->GetLaneType() ==
                      XodrLaneType::DRIVING;
    bool filtered_e =
        (*g)[ed].edge_type == XodrLaneEdgeType::LANE_SUCCESSOR_EDGE;

    bool filtered = filtered_s && filtered_t && filtered_e;
    return filtered;
  }

  bool operator()(XodrLaneGraph::vertex_descriptor vd) const {
    bool filtered = (*g)[vd].lane->GetLaneType() == XodrLaneType::DRIVING;
    return filtered;
  }
  XodrLaneGraph* g;
};

struct TypeDriving {  // both edge and vertex
  bool operator()(XodrLaneGraph::edge_descriptor ed) const {
    bool filtered_s = (*g)[boost::source(ed, *g)].lane->GetLaneType() ==
                      XodrLaneType::DRIVING;
    bool filtered_t = (*g)[boost::target(ed, *g)].lane->GetLaneType() ==
                      XodrLaneType::DRIVING;

    bool filtered = filtered_s && filtered_t;
    return filtered;
  }

  bool operator()(XodrLaneGraph::vertex_descriptor vd) const {
    bool filtered = (*g)[vd].lane->GetLaneType() == XodrLaneType::DRIVING;
    return filtered;
  }
  XodrLaneGraph* g;
};

struct EdgeTypeRoadSuccessor {  // both edge and vertex
  bool operator()(XodrLaneGraph::edge_descriptor ed) const {
    bool filtered_e =
        (*g)[ed].edge_type == XodrLaneEdgeType::ROAD_SUCCESSOR_EDGE;

    bool filtered = filtered_e;
    return filtered;
  }

  bool operator()(XodrLaneGraph::vertex_descriptor vd) const { return true; }
  XodrLaneGraph* g;
};

template <typename Predicate>
using FilteredXodrLaneGraph_t =
    boost::filtered_graph<XodrLaneGraph, Predicate, Predicate>;

typedef FilteredXodrLaneGraph_t<TypeDrivingAndEdgeTypeLaneSuccessor>
    FilteredXodrLaneGraph;

class Roadgraph {
 public:
  Roadgraph() {}

  XodrLaneId AddLane(const XodrRoadId& road_id, const XodrLanePtr& laneptr);

  bool AddInnerNeighbor(const XodrLaneId& inner_id,
                          const XodrLaneId& outer_id);

  bool AddOuterNeighbor(const XodrLaneId& inner_id,
                          const XodrLaneId& outer_id);

  bool AddLaneSuccessor(const XodrLaneId& prev, const XodrLaneId& succ);

  bool AddRoadSuccessor(const XodrLaneId& prev, const XodrLaneId& succ);

  std::vector<XodrLaneId> GetSuccessorLanes(const XodrLaneId& lane_id) const;

  std::vector<XodrLaneId> GetPredecessorLanes(
      const XodrLaneId& lane_id) const;

  template <class Predicate>
  bool CheckIdInFilteredGraph(const FilteredXodrLaneGraph_t<Predicate>& fg,
                                  const XodrLaneId& lane_id) const;

  template <class Predicate>
  std::vector<XodrLaneId> FindPath(const XodrLaneId& startid,
                                    const XodrLaneId& goalid);

  std::vector<XodrRoadId> FindRoadPath(const XodrRoadId& startid,
                                         const XodrRoadId& goalid);

  std::vector<XodrLaneId> FindDrivableLanePath(const XodrLaneId& startid,
                                                  const XodrLaneId& goalid);

  std::vector<std::vector<XodrLaneId>> FindAllPathsInSubgraph(
      const std::vector<XodrLaneEdgeType>& edge_type_subset,
      const std::vector<XodrLaneId>& lane_id_subset);

  XodrLanePtr GetLanePtr(const XodrLaneId& id) const;

  std::vector<XodrLaneId> GetAllLaneids() const;

  std::pair<XodrLaneId, bool> GetPlanViewForRoadId(const XodrRoadId& id) const;

  std::pair<std::vector<XodrDrivingDirection>, bool>
          GetDrivingDirectionsForRoadId(const XodrRoadId &id) const;

  //! outer_lane_id can be some outer_lane_id (position 1,2,3)
  std::pair<XodrLaneId, bool> GetPlanViewForLaneId(
      const XodrLaneId& outer_lane_id) const;

  // TODO: Move to XodrLaneGraph, but XodrLaneGraph is not wrapped to python
  PolygonPtr GetLanePolygonForLaneId(const XodrLaneId& lane_id);

  XodrRoadId GetRoadForLaneId(const XodrLaneId& lane_id);

  //! XodrLaneId of the neighboring lane and a flag if it exists or not
  std::pair<XodrLaneId, bool> GetInnerNeighbor(
      const XodrLaneId& lane_id) const;

  //! XodrLaneId of the neighboring lane and a flag if it exists or not
  //! @note make sure to not call this function on the planview lane!
  std::pair<XodrLaneId, bool> GetOuterNeighbor(
      const XodrLaneId& lane_id) const;

  //! XodrLaneIds of all neighboring lanes in the same driving direction.
  //! Includes neighbors of neighbors
  //! @note cannot be called with the planview lane!
  std::vector<XodrLaneId> GetAllNeighbors(const XodrLaneId& lane_id) const;

  bool HasLane(const XodrLaneId& lane_id) const;

  void PrintGraph(const char* filename);

  void PrintGraph(std::ofstream& dotfile);

  XodrLaneGraph GetLaneGraph() const { return g_; }

  XodrLaneVertex GetVertex(vertex_t v_des) const { return g_[v_des]; }

  std::vector<vertex_t> GetVertices() const;

  XodrLaneEdge GetEdge(edge_t e_des) const { return g_[e_des]; }

  std::vector<vertex_t> GetNextVertices(vertex_t current_vertex) const;

  std::vector<edge_t> GetOutEdges(vertex_t current_vertex) const;

  std::vector<edge_t> GetEdges() const;

  edge_t GetEdgeDescr(vertex_t from, vertex_t to) const;

  std::pair<vertex_t, bool> GetVertexByLaneId(
      const XodrLaneId& lane_id) const;

  void GenerateVertices(OpenDriveMapPtr map);

  void GeneratePreAndSuccessors(OpenDriveMapPtr map);

  void GenerateNeighbours(OpenDriveMapPtr map);

  void GenerateFromJunctions(OpenDriveMapPtr map);

  void GeneratePolygonsForVertices();

  void Generate(OpenDriveMapPtr map);

  std::pair<XodrLanePtr, XodrLanePtr> ComputeXodrLaneBoundaries(
      const XodrLaneId& lane_id) const;

  std::pair<std::vector<XodrLanePtr>, std::vector<XodrLanePtr>>
  ComputeRouteBoundaries(const std::vector<XodrLaneId>& horizon) const;

  std::pair<PolygonPtr, bool> ComputeXodrLanePolygon(
      const XodrLaneId& lane_id) const;
  std::vector<std::pair<XodrLaneId, bool>> GetOuterNeighborsPlanview(
    const XodrLaneId &lane_id) const;

  //! Roadgraph extension
  std::pair<XodrLaneId, bool> GetLeftLane(const XodrLaneId& lane_id,
    const XodrDrivingDirection& driving_direction);
  std::pair<XodrLaneId, bool> GetRightLane(const XodrLaneId& lane_id,
    const XodrDrivingDirection& driving_direction);
  std::pair<XodrLaneId, bool> GetNextLane(
    const std::vector<XodrRoadId>& road_ids,
    const XodrLaneId& lane_id);
  std::pair<XodrLaneId, bool> GetLeftBoundary(const XodrLaneId& lane_id,
    const XodrDrivingDirection& driving_direction);
  std::pair<XodrLaneId, bool> GetRightBoundary(const XodrLaneId& lane_id,
    const XodrDrivingDirection& driving_direction);

 private:
  XodrLaneGraph g_;

  bool AddEdgeOfType(const XodrLaneId& source_id,
                        const XodrLaneId& tarGetId,
                        const XodrLaneEdgeType& edgetype);

  std::vector<std::pair<XodrLaneId, bool>> GetNeighborFromEdgetype(
      const XodrLaneId& lane_id, const XodrLaneEdgeType edge_type) const;

  template <class XodrRoadIdMap, class XodrLaneIdMap, class XodrLaneMap>
  class my_vertex_writer_graph {
   public:
    my_vertex_writer_graph(XodrRoadIdMap r, XodrLaneIdMap l,
                           XodrLaneMap lanemap)
        : rm(r), lm(l), lanemap_(lanemap) {}
    template <class XodrLaneVertex>
    void operator()(std::ostream& out, const XodrLaneVertex& s) const {
      out << "["
          << "label=\""
          << "road_id=" << rm[s] << " lane_id=" << lm[s]
          << " lane_pos=" << lanemap_[s]->GetLanePosition() << "\"]";
    }

   private:
    XodrRoadIdMap rm;
    XodrLaneIdMap lm;
    XodrLaneMap lanemap_;
  };

  template <class XodrRoadIdMap, class XodrLaneIdMap, class XodrLaneMap>
  inline my_vertex_writer_graph<XodrRoadIdMap, XodrLaneIdMap, XodrLaneMap>
  make_vertex_label_writer_graph(XodrRoadIdMap r, XodrLaneIdMap l,
                                 XodrLaneMap lm) {
    return my_vertex_writer_graph<XodrRoadIdMap, XodrLaneIdMap, XodrLaneMap>(
        r, l, lm);
  }

  template <class TypeMap>
  class my_edge_writer_text {
   public:
    my_edge_writer_text(TypeMap t) : tm(t) {}
    template <class XodrLaneEdge>
    void operator()(std::ostream& out, const XodrLaneEdge& e) const {
      const char* color;
      if (tm[e] == 0) {
        color = "red";
      } else if (tm[e] == 3) {
        color = "green";
      } else {
        color = "blue";
      }
      // const char* color = tm[e]==0?"red":"blue";
      out << "[label=\"" << tm[e] << "\""
          << "color=\"" << color << "\""
          << "]";
    }

   private:
    TypeMap tm;
  };

  template <class TypeMap>
  inline my_edge_writer_text<TypeMap> make_edge_label_writer_text(TypeMap t) {
    return my_edge_writer_text<TypeMap>(t);
  }
};

typedef std::shared_ptr<Roadgraph> RoadgraphPtr;

}  // namespace map
}  // namespace world
}  // namespace modules

#include "roadgraph_impl.hpp"

#endif  // MODULES_WORLD_ROADGRAPH_HPP_
