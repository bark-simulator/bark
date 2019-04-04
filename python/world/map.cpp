// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include <string>
#include "map.hpp"
#include "modules/world/map/map_interface.hpp"
#include "modules/world/map/local_map.hpp"
#include "modules/world/map/roadgraph.hpp"
#include "modules/world/opendrive/opendrive.hpp"

using namespace modules::world::map;
using modules::world::opendrive::LaneId;


void python_map(py::module m) {
  py::class_<MapInterface, std::shared_ptr<MapInterface>>(m, "MapInterface")
      .def(py::init<>())
      .def("set_open_drive_map", &MapInterface::set_open_drive_map)
      .def("get_nearest_lanes", &MapInterface::get_nearest_lanes)
      .def("set_roadgraph", &MapInterface::set_roadgraph)
      .def("get_roadgraph", &MapInterface::get_roadgraph)
      .def("get_open_drive_map", &MapInterface::get_open_drive_map);

  py::class_<LocalMap, std::shared_ptr<LocalMap>>(m, "LocalMap")
      .def(py::init<LaneId, const MapInterfacePtr&>())
      .def_property_readonly("inner_line", &LocalMap::get_inner_line)
      .def_property_readonly("outer_line", &LocalMap::get_outer_line)
      .def_property_readonly("center_line", &LocalMap::get_center_line)
      .def("set_goal_lane_id", &LocalMap::set_goal_lane_id)
      .def("set_map_interface", &LocalMap::set_map_interface)
      .def("generate", &LocalMap::generate);

  py::class_<Roadgraph, std::shared_ptr<Roadgraph>>(m, "Roadgraph")
    .def(py::init<>())
    .def("add_lane", &Roadgraph::add_lane)
    .def("get_vertices", &Roadgraph::get_vertices)
    .def("get_edges", &Roadgraph::get_edges)
    .def("get_edge", &Roadgraph::get_edge)
    .def("get_edge_descr", &Roadgraph::get_edge_descr)
    .def("get_out_edges", &Roadgraph::get_out_edges)
    .def("get_vertex", &Roadgraph::get_vertex)
    .def("get_next_vertices", &Roadgraph::get_next_vertices)
    .def("get_vertex_by_lane_id", &Roadgraph::get_vertex_by_lane_id)
    .def("add_inner_neighbor", &Roadgraph::add_inner_neighbor)
    .def("get_inner_neighbor", &Roadgraph::get_inner_neighbor)
    .def("add_outer_neighbor", &Roadgraph::add_outer_neighbor)
    .def("find_path", &Roadgraph::find_path)
    .def("print_graph", (void (Roadgraph::*)(const char*)) &Roadgraph::print_graph)
    .def("add_successor", &Roadgraph::add_successor);
}
