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
#include "modules/world/map/local_map.hpp"
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
      .def("GenerateRoadCorridor", py::overload_cast<const std::vector<XodrRoadId>&,
                                  const XodrDrivingDirection&>(&MapInterface::GenerateRoadCorridor))
      .def("GetRoadCorridor", &MapInterface::GetRoadCorridor)
      .def("get_lane", &MapInterface::get_lane)
      .def("get_all_corridors", &MapInterface::get_all_corridors)
      .def("compute_driving_corridor_from_start_to_goal", &MapInterface::ComputeDrivingCorridorFromStartToGoal)
      .def("compute_all_driving_corridors", &MapInterface::ComputeAllDrivingCorridors)
      .def("compute_all_path_boundaries", &MapInterface::ComputeAllPathBoundaries)
      .def("get_adjacent_corridors_same_direction", &MapInterface::GetAdjacentDrivingCorridorsSameDirection)
      .def("get_splitting_corridors", &MapInterface::GetSplittingDrivingCorridors)
      .def("line_segment_inside_corridor", &MapInterface::LineSegmentInsideCorridor)
      .def("find_lane", &MapInterface::FindXodrLane)
      .def("has_correct_driving_direction", &MapInterface::HasCorrectDrivingDirection)
      //.def("compute_lane_boundaries_horizon", &MapInterface::ComputeXodrLaneBoundariesHorizon)
      /*.def("calculate_driving_corridor",[](const MapInterface& m, const XodrLaneId& startid, const XodrLaneId goalid) {
          Line inner_line, outer_line, center_line;
          bool result = m.CalculateDrivingCorridor(startid, goalid, inner_line, outer_line, center_line);
          return std::make_tuple(inner_line, outer_line, center_line);
      })*/
      ;

  py::class_<DrivingCorridor,
             std::shared_ptr<DrivingCorridor>>(m, "DrivingCorridor")
      .def(py::init<>())
      .def_property("inner", &DrivingCorridor::get_inner,
        &DrivingCorridor::set_inner)
      .def_property("outer", &DrivingCorridor::get_outer,
        &DrivingCorridor::set_outer)
      .def_property("center", &DrivingCorridor::get_center,
        &DrivingCorridor::set_center)
      .def("get_lane_ids", &DrivingCorridor::get_lane_ids)
      .def(py::pickle(
      [](const DrivingCorridor& d) -> py::tuple { // __getstate__
          /* Return a tuple that fully encodes the state of the object */
          return py::make_tuple(d.get_inner(), d.get_outer(), d.get_center(), d.computed);
      },
      [](const py::tuple &t)  { // __setstate__
          if (t.size() != 4)
              throw std::runtime_error("Invalid driving corridor state!");

          DrivingCorridor d;
          d.set_inner(t[0].cast<Line>());
          d.set_outer(t[1].cast<Line>());
          d.set_center(t[2].cast<Line>());
          d.computed = t[3].cast<bool>();
          return d;
      }));

  py::class_<LocalMap, std::shared_ptr<LocalMap>>(m, "LocalMap")
      .def(py::init<const GoalDefinitionPtr&, const MapInterfacePtr&>())
      .def("set_goal_lane_id", &LocalMap::set_goal_lane_id)
      .def("get_horizon_driving_corridor",
        &LocalMap::get_horizon_driving_corridor)
      .def("get_driving_corridor", &LocalMap::get_driving_corridor)
      .def("set_map_interface", &LocalMap::set_map_interface)
      .def("generate", &LocalMap::Generate)
      .def(py::pickle(
        [](const LocalMap& l) -> py::tuple { // __getstate__
            /* Return a tuple that fully encodes the state of the object */
            return py::make_tuple(l.get_goal_lane_id(),
                                  goal_definition_to_python(l.get_goal_definition()),
                                  l.get_driving_corridor());
        },
        [](py::tuple &t)  { // __setstate__
            if (t.size() != 3)
                throw std::runtime_error("Invalid local map state!");

            return new LocalMap(t[0].cast<XodrLaneId>(),
                 python_to_goal_definition(t[1].cast<py::tuple>()),
                 t[2].cast<DrivingCorridor>());
        }));

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
    .def("print_graph", (void (Roadgraph::*)(const char*)) &Roadgraph::print_graph)
    .def("add_lane_successor", &Roadgraph::add_lane_successor)
    .def("Generate", &Roadgraph::Generate)
    .def("GetLanePolygonForLaneId", &Roadgraph::GetLanePolygonForLaneId)
    .def("GetRoadForLaneId", &Roadgraph::GetRoadForLaneId)
    .def("compute_lane_boundaries", &Roadgraph::ComputeXodrLaneBoundaries);  // GetRoadForLaneId



//! RoadCorridor
py::class_<RoadCorridor,
           std::shared_ptr<RoadCorridor>>(m, "RoadCorridor")
    .def(py::init<>())
    .def_property("roads", &RoadCorridor::GetRoads,
      &RoadCorridor::SetRoads)
    .def("lanes", &RoadCorridor::GetLanes)
    .def("get_road", &RoadCorridor::GetRoad)
    .def("get_lane_corridor", &RoadCorridor::GetLaneCorridor)
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
    .def_property_readonly("right_boundary", &LaneCorridor::GetRightBoundary);

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
  .def_property_readonly("polygon", &Lane::GetPolygon);


// TODO(@hart): make pickable -> requires XODR to be as well
py::class_<Road,
           std::shared_ptr<Road>>(m, "Road")
  .def(py::init<XodrRoadPtr>())
  .def_property_readonly("next_road", &Road::GetNextRoad)
  .def_property_readonly("lanes", &Road::GetLanes)
  .def("get_lane", &Road::GetLane);
}
