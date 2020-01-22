// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include <string>
#include <vector>
#include <map>
#include "map.hpp"

#include "python/polymorphic_conversion.hpp"
#include "modules/world/map/map_interface.hpp"
#include "modules/world/map/roadgraph.hpp"
#include "modules/world/opendrive/opendrive.hpp"
#include "modules/geometry/geometry.hpp"

using namespace modules::world::map;
using modules::world::opendrive::XodrLaneId;
using modules::geometry::Point2d;
using modules::geometry::Line;

void python_map(py::module m) {
  py::class_<MapInterface, std::shared_ptr<MapInterface>>(m, "MapInterface")
    .def(py::init<>())
    .def("set_open_drive_map", &MapInterface::set_open_drive_map)
    .def("find_nearest_lanes", [](const MapInterface& m,
                                  const Point2d& point,
                                  const unsigned& num_lanes) {
        std::vector<XodrLanePtr> lanes;
        m.FindNearestXodrLanes(point, num_lanes, lanes);
        return lanes;
    })
    .def("set_roadgraph", &MapInterface::set_roadgraph)
    .def("get_roadgraph", &MapInterface::get_roadgraph)
    .def("get_open_drive_map", &MapInterface::get_open_drive_map)
      .def("GenerateRoadCorridor", py::overload_cast<const std::vector<XodrRoadId>&,
                                  const XodrDrivingDirection&>(&MapInterface::GenerateRoadCorridor))
      .def("GenerateRoadCorridor", py::overload_cast<const modules::geometry::Point2d&,
                                  const modules::geometry::Polygon&>(&MapInterface::GenerateRoadCorridor))
    .def("GetRoadCorridor", &MapInterface::GetRoadCorridor)
    .def("get_lane", &MapInterface::get_lane)
    .def("compute_all_path_boundaries",
      &MapInterface::ComputeAllPathBoundaries)
    .def("find_lane", &MapInterface::FindXodrLane);
 
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
    .def("get_all_laneids", &Roadgraph::get_all_laneids)
    .def("get_inner_neighbor", &Roadgraph::get_inner_neighbor)
    .def("add_outer_neighbor", &Roadgraph::add_outer_neighbor)
    .def("FindDrivableLanePath", &Roadgraph::FindDrivableLanePath)
    .def("FindRoadPath", &Roadgraph::FindRoadPath)
    .def("print_graph",
      (void (Roadgraph::*)(const char*)) &Roadgraph::print_graph)
    .def("add_lane_successor", &Roadgraph::add_lane_successor)
    .def("Generate", &Roadgraph::Generate)
    .def("GetLanePolygonForLaneId", &Roadgraph::GetLanePolygonForLaneId)
    .def("GetRoadForLaneId", &Roadgraph::GetRoadForLaneId)
    .def("compute_lane_boundaries",
      &Roadgraph::ComputeXodrLaneBoundaries);  // GetRoadForLaneId



//! RoadCorridor
py::class_<RoadCorridor,
           std::shared_ptr<RoadCorridor>>(m, "RoadCorridor")
    .def(py::init<>())
    .def_property("roads", &RoadCorridor::GetRoads,
      &RoadCorridor::SetRoads)
    .def("lanes", &RoadCorridor::GetLanes)
    .def("get_road", &RoadCorridor::GetRoad)
    .def("get_lane_corridor", &RoadCorridor::GetLaneCorridor)
    .def("GetCurrentLaneCorridor", &RoadCorridor::GetCurrentLaneCorridor)
    .def_property_readonly("lane_corridors",
      &RoadCorridor::GetUniqueLaneCorridors)
    .def(py::pickle(
    [](const RoadCorridor& rc) -> py::tuple {  // __getstate__
        /* Return a tuple that fully encodes the state of the object */
        return py::make_tuple(rc.GetRoads(),
                              rc.GetUniqueLaneCorridors(),
                              rc.GetLaneCorridorMap());
    },
    [](const py::tuple &t) {  // __setstate__
        if (t.size() != 3)
          throw std::runtime_error("Invalid RoadCorridor state!");
        RoadCorridor rc;
        rc.SetRoads(t[0].cast<Roads>());
        rc.SetUniqueLaneCorridors(
          t[1].cast<std::vector<LaneCorridorPtr>>());
        rc.SetLaneCorridorMap(
          t[2].cast<std::map<LaneId, LaneCorridorPtr>>());
        return rc;
    }));

//! LaneCorridor
// TODO(@hart): make pickable -> requires XODR to be as well
py::class_<LaneCorridor,
           std::shared_ptr<LaneCorridor>>(m, "LaneCorridor")
    .def(py::init<>())
    .def_property_readonly("polygon", &LaneCorridor::GetMergedPolygon)
    .def_property_readonly("center_line", &LaneCorridor::GetCenterLine)
    .def_property_readonly("left_boundary", &LaneCorridor::GetLeftBoundary)
    .def_property_readonly("right_boundary", &LaneCorridor::GetRightBoundary)
    .def("__eq__", &LaneCorridor::operator== )
    .def("__neq__", &LaneCorridor::operator!=);

py::class_<Boundary,
           std::shared_ptr<Boundary>>(m, "Boundary")
  .def(py::init<>())
  .def_property_readonly("line", &Boundary::GetLine)
  .def_property_readonly("type", &Boundary::GetType);

// TODO(@hart): make pickable -> requires XODR to be as well
py::class_<Lane,
           std::shared_ptr<Lane>>(m, "Lane")
  .def(py::init<XodrLanePtr>())
  .def_property_readonly("center_line", &Lane::GetCenterLine)
  .def_property_readonly("right_lane", &Lane::GetRightLane)
  .def_property_readonly("left_lane", &Lane::GetLeftLane)
  .def_property_readonly("left_boundary", &Lane::GetLeftBoundary)
  .def_property_readonly("right_boundary", &Lane::GetRightBoundary)
  .def_property_readonly("next_lane", &Lane::GetNextLane)
  .def_property_readonly("lane_id", &Lane::get_id)
  .def_property_readonly("type", &Lane::get_lane_type)
  .def_property_readonly("driving_direction", &Lane::get_driving_direction)
  .def_property_readonly("lane_position", &Lane::get_lane_position)
  .def_property_readonly("polygon", &Lane::GetPolygon);


// TODO(@hart): make pickable -> requires XODR to be as well
py::class_<Road,
           std::shared_ptr<Road>>(m, "Road")
  .def(py::init<XodrRoadPtr>())
  .def_property_readonly("next_road", &Road::GetNextRoad)
  .def_property_readonly("road_id", &Road::get_id)
  .def_property_readonly("lanes", &Road::GetLanes)
  .def("get_lane", &Road::GetLane);
}
