// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_ROADGRAPH_IMPL_HPP_
#define BARK_WORLD_ROADGRAPH_IMPL_HPP_

namespace bark {
namespace world {
namespace map {

template <class Predicate>
bool Roadgraph::CheckIdInFilteredGraph(
    const FilteredXodrLaneGraph_t<Predicate>& fg,
    const XodrLaneId& lane_id) const {
  typename boost::graph_traits<
      FilteredXodrLaneGraph_t<Predicate>>::vertex_iterator i,
      end;
  for (boost::tie(i, end) = boost::vertices(fg); i != end; ++i) {
    if (fg[*i].global_lane_id == lane_id) {
      return true;
    }
  }
  return false;
}

template <class Predicate>
std::vector<XodrLaneId> Roadgraph::FindPath(const XodrLaneId& startid,
                                            const XodrLaneId& goalid) {
  std::vector<XodrLaneId> path;

  std::pair<vertex_t, bool> start_vertex = GetVertexByLaneId(startid);
  std::pair<vertex_t, bool> goal_vertex = GetVertexByLaneId(goalid);

  // filter graph
  Predicate predicate{&g_};
  FilteredXodrLaneGraph_t<Predicate> fg(g_, predicate, predicate);
  bool start_goal_valid_in_fg =
      CheckIdInFilteredGraph<Predicate>(fg, startid) &&
      CheckIdInFilteredGraph<Predicate>(fg, goalid);

  if (start_vertex.second && goal_vertex.second && start_goal_valid_in_fg) {
    // std::cout << "start_vertex: " << start_vertex.first << " id " <<
    // g_[start_vertex.first].lane->GetId()<< std::endl; std::cout <<
    // "goal_vertex: " << goal_vertex.first << " id " <<
    // g_[goal_vertex.first].lane->GetId() <<std::endl; std::cout <<
    // "goal_vertex type: " << g_[goal_vertex.first].lane->GetLaneType() <<
    // std::endl; std::cout << "goal_vertex type: " <<
    // fg[goal_vertex.first].lane->GetLaneType() << std::endl;

    size_t num_vertices = boost::num_vertices(fg);

    std::vector<vertex_t> p(num_vertices);

    std::vector<int> d(num_vertices);

    boost::property_map<FilteredXodrLaneGraph, float XodrLaneEdge::*>::type
        weightmap = boost::get(&XodrLaneEdge::weight, fg);

    boost::dijkstra_shortest_paths(
        fg, start_vertex.first,
        predecessor_map(boost::make_iterator_property_map(
                            p.begin(), get(boost::vertex_index, fg)))
            .distance_map(boost::make_iterator_property_map(
                d.begin(), get(boost::vertex_index, fg)))
            .weight_map(weightmap));

    // get shortest path from predecessor map
    int stop_the_loop = num_vertices;
    int idx = 0;
    boost::graph_traits<XodrLaneGraph>::vertex_descriptor current =
        goal_vertex.first;
    while (current != start_vertex.first && idx < stop_the_loop) {
      path.push_back(fg[current].global_lane_id);
      if (current == p[current]) {
        return std::vector<XodrLaneId>();
      }
      current = p[current];
      ++idx;
    }
    path.push_back(fg[start_vertex.first].global_lane_id);
    std::reverse(path.begin(), path.end());
  }
  return path;
}

}  // namespace map
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_ROADGRAPH_IMPL_HPP_
