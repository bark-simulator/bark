// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/world/map/roadgraph.hpp"

namespace modules {
namespace world {
namespace map {

void Roadgraph::GenerateVertices(OpenDriveMapPtr map)
{
  for (auto const &road_element : map->get_roads())
  { // std::map
    for (auto const &lane_section_element : road_element.second->get_lane_sections())
    { // std::vector
      for (auto const &lane_element : lane_section_element->get_lanes())
      { // std::map
        add_lane(road_element.first, lane_element.second);
      }
    }
  }
}

void Roadgraph::GeneratePreAndSuccessors(OpenDriveMapPtr map) 
{
  // add successors, predecessors
  for (auto const &road_element : map->get_roads())
  {                                                                                     // std::map
    RoadId successor_road_id = road_element.second->get_link().get_successor().id_;     // this is the position!!!!!! (-4, 4)
    RoadId predecessor_road_id = road_element.second->get_link().get_predecessor().id_; // this is the position!!!!!! (-4, 4)
    if (successor_road_id > 1000)
    {
      continue;
    }

    RoadPtr successor_road = map->get_roads().at(successor_road_id);
    LaneSectionPtr successor_lane_section = successor_road->get_lane_sections().front();

    // make sure that there is a predecessor!!
    LaneSectionPtr predecessor_lane_section = nullptr;

    // TODO: That's pretty ugly, move check for road_id to map
    try {
    auto iter(map->get_roads().lower_bound(predecessor_road_id));
    if (iter == map->get_roads().end() || predecessor_road_id < iter->first) {    // not found
      RoadPtr predecessor_road = map->get_roads().at(predecessor_road_id);
      predecessor_lane_section = predecessor_road->get_lane_sections().back();
    }
    else {
      std::cerr << "Road has no predeseccor road. \n";
    }
    }
    catch (...) {

    }

    // TODO (@hart): there could be mult. lane_sections
    for (auto const &lane_section_element : road_element.second->get_lane_sections())
    { // std::vector
      for (auto const &lane_element : lane_section_element->get_lanes())
      { // std::map

        // add successor edge
        LanePosition successor_lane_position = lane_element.second->get_link().to_position;
        if (successor_lane_position == 0)
        {
          continue;
        }

        LanePtr successor_lane = successor_lane_section->get_lane_by_position(successor_lane_position);
        // LanePtr successor_lane = lane_section_element->get_lanes().at(successor_lane_position);

        if (successor_lane)
        {
          bool success = add_successor(lane_element.first, successor_lane->get_id());
        }

        // does not always have predecessor
        try
        {
          // search for predecessor_lane_position in previos lane section
          if (predecessor_lane_section)
          {
            LanePosition predecessor_lane_position = lane_element.second->get_link().from_position;
            if (predecessor_lane_position == 0)
            {
              continue;
            }
            LanePtr predecessor_lane = predecessor_lane_section->get_lane_by_position(predecessor_lane_position);
            // if found add; convert predecessor to successor
            if (predecessor_lane)
            {
              bool success = add_successor(predecessor_lane->get_id(), lane_element.first);
            }
          }
        }
        catch (const std::exception &ex)
        {
          std::cerr << "Road has no predeseccor road. \n";
        }
      }
    }
  }
}

void Roadgraph::GenerateNeighbours(OpenDriveMapPtr map)
{
  // add neighbor edges
  for (auto const &road_element : map->get_roads())
  { // std::map
    for (auto const &lane_section_element : road_element.second->get_lane_sections())
    { // std::vector
      for (auto const &lane_element : lane_section_element->get_lanes())
      { // std::map
        if (lane_element.second->get_lane_position() != 0)
        {
          LanePosition inner_lane_pos;
          if (lane_element.second->get_lane_position() > 0)
          {
            inner_lane_pos = lane_element.second->get_lane_position() - 1;
          }
          else
          {
            inner_lane_pos = lane_element.second->get_lane_position() + 1;
          }

          LanePtr inner_lane = lane_section_element->get_lane_by_position(inner_lane_pos);
          if (inner_lane)
          {
            bool success_inner = add_inner_neighbor(inner_lane->get_id(), lane_element.second->get_id());
            bool success_outer = add_outer_neighbor(inner_lane->get_id(), lane_element.second->get_id());
          }
        }
      }
    }
  }
}

void Roadgraph::GenerateFromJunctions(OpenDriveMapPtr map)
{
    // map junctions
  for (auto const &road_element : map->get_junctions())
  { // std::map
    for (auto const &connection_element : road_element.second->get_connections())
    { // std::map

      RoadPtr incoming_road = map->get_road(connection_element.second.incoming_road_);
      RoadPtr connecting_road = map->get_road(connection_element.second.connecting_road_);
      LaneSectionPtr pre_lane_section = incoming_road->get_lane_sections().front();
      LaneSectionPtr successor_lane_section = connecting_road->get_lane_sections().front();
      for (auto const &lane_link_element : connection_element.second.get_lane_links())
      {
        // add successor edge
        if (pre_lane_section && successor_lane_section)
        {
          try
          {
            LanePtr pre_lane = pre_lane_section->get_lane_by_position(lane_link_element.from_position);
            LanePtr successor_lane = successor_lane_section->get_lane_by_position(lane_link_element.to_position);
            if (pre_lane && successor_lane)
            {
              bool success = add_successor(pre_lane->get_id(), successor_lane->get_id());
            }
          }
          catch (...)
          {
            std::cerr << "Junction has no connections. \n";
          }
        }
      }
    }
  }
}

void Roadgraph::Generate(OpenDriveMapPtr map)
{
  GenerateVertices(map);

  GeneratePreAndSuccessors(map);

  GenerateNeighbours(map);

  GenerateFromJunctions(map);
}

std::pair<LanePtr, LanePtr> Roadgraph::ComputeLaneBoundaries(const LaneId &lane_id)
{
  LanePtr inner, outer;
  std::pair<vertex_t, bool> v = get_vertex_by_lane_id(lane_id);
  auto l = get_lane_graph()[v.first].lane;
  assert(l->get_lane_position() != 0); // make sure we are not at the planview, as a driving corridor cannot be computed from here.
  outer = l;

  std::pair<LaneId, bool> innerid = get_inner_neighbor(lane_id);
  if (innerid.second)
  {
    std::pair<vertex_t, bool> v_inner = get_vertex_by_lane_id(innerid.first);
    if (v_inner.second)
    {
      inner = get_lane_graph()[v_inner.first].lane;
    }
    else
    {
      inner = NULL;
    }
  }
  else
  { //you are probably at the planview and do not have inner lanes?
  }
  return std::make_pair(inner, outer);
}

// TODO Klemens: change to LanePtr to Line
std::pair<std::vector<LanePtr>, std::vector<LanePtr>> Roadgraph::ComputeRouteBoundaries(const std::vector<LaneId> &horizon)
{
  std::vector<LanePtr> inner, outer;
  if (!horizon.empty())
  {
    for (auto &h : horizon)
    {
      auto lane_boundaries = ComputeLaneBoundaries(h);
      inner.push_back(lane_boundaries.first);
      outer.push_back(lane_boundaries.second);
    }
  }
  return std::make_pair(inner, outer);
}

void Roadgraph::ComputeLanePolygon(const LaneId &lane_id)
{

  std::pair<LanePtr, LanePtr> lb = ComputeLaneBoundaries(lane_id);

  PolygonPtr polygon = std::make_shared<modules::geometry::Polygon>();
  // inner
  for (auto const &p : lb.first->get_line())
  {
    polygon->add_point(p);
  }
  // outer
  for (auto const &p : lb.second->get_line())
  {
    polygon->add_point(p);
  }
}

} // namespace map
} // namespace world
} // namespace modules
