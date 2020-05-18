// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include <string>
#include <memory>
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
    .def("SetOpenDriveMap", &MapInterface::SetOpenDriveMap)
    .def("find_nearest_lanes", [](const MapInterface& m,
                                  const Point2d& point,
                                  const unsigned& num_lanes) {
        std::vector<XodrLanePtr> lanes;
        m.FindNearestXodrLanes(point, num_lanes, lanes);
        return lanes;
    })
    .def("SetRoadgraph", &MapInterface::SetRoadgraph)
    .def("GetRoadgraph", &MapInterface::GetRoadgraph)
    .def("GetOpenDriveMap", &MapInterface::GetOpenDriveMap)
    .def("GenerateRoadCorridor",
      py::overload_cast<const std::vector<XodrRoadId>&,
      const XodrDrivingDirection&>(&MapInterface::GenerateRoadCorridor))
    .def("GenerateRoadCorridor",
      py::overload_cast<const modules::geometry::Point2d&,
      const modules::geometry::Polygon&>(&MapInterface::GenerateRoadCorridor))
    .def("GetRoadCorridor", &MapInterface::GetRoadCorridor)
    .def("GetLane", &MapInterface::GetLane)
    .def("ComputeAllPathBoundaries",
      &MapInterface::ComputeAllPathBoundaries)
    .def("FindLane", &MapInterface::FindXodrLane);

  py::class_<Roadgraph, std::shared_ptr<Roadgraph>>(m, "Roadgraph")
    .def(py::init<>())
    .def("AddLane", &Roadgraph::AddLane)
    .def("GetVertices", &Roadgraph::GetVertices)
    .def("GetEdges", &Roadgraph::GetEdges)
    .def("GetEdge", &Roadgraph::GetEdge)
    .def("GetEdgeDescr", &Roadgraph::GetEdgeDescr)
    .def("GetOutEdges", &Roadgraph::GetOutEdges)
    .def("GetVertex", &Roadgraph::GetVertex)
    .def("GetNextVertices", &Roadgraph::GetNextVertices)
    .def("GetVertexByLaneId", &Roadgraph::GetVertexByLaneId)
    .def("AddInnerNeighbor", &Roadgraph::AddInnerNeighbor)
    .def("GetAllLaneids", &Roadgraph::GetAllLaneids)
    .def("GetInnerNeighbor", &Roadgraph::GetInnerNeighbor)
    .def("AddOuterNeighbor", &Roadgraph::AddOuterNeighbor)
    .def("FindDrivableLanePath", &Roadgraph::FindDrivableLanePath)
    .def("FindRoadPath", &Roadgraph::FindRoadPath)
    .def("PrintGraph",
      (void (Roadgraph::*)(const char*)) &Roadgraph::PrintGraph)
    .def("AddLaneSuccessor", &Roadgraph::AddLaneSuccessor)
    .def("Generate", &Roadgraph::Generate)
    .def("GetLanePolygonForLaneId", &Roadgraph::GetLanePolygonForLaneId)
    .def("GetRoadForLaneId", &Roadgraph::GetRoadForLaneId)
    .def("GetDrivingDirectionsForRoadId", &Roadgraph::GetDrivingDirectionsForRoadId)
    .def("compute_lane_boundaries",
      &Roadgraph::ComputeXodrLaneBoundaries);  // GetRoadForLaneId


py::class_<RoadCorridor,
           std::shared_ptr<RoadCorridor>>(m, "RoadCorridor")
  .def(py::init<>())
  .def_property("roads", &RoadCorridor::GetRoads,
    &RoadCorridor::SetRoads)
  .def_property_readonly("lanes", &RoadCorridor::GetLanes)
  .def_property_readonly("road_ids", &RoadCorridor::GetRoadIds)
  .def("GetRoad", &RoadCorridor::GetRoad)
  .def_property_readonly("polygon", &RoadCorridor::GetPolygon)
  .def("GetLaneCorridor", &RoadCorridor::GetLaneCorridor)
  .def("GetCurrentLaneCorridor", &RoadCorridor::GetCurrentLaneCorridor)
  .def("GetLeftRightLaneCorridor", &RoadCorridor::GetLeftRightLaneCorridor)
  .def_property_readonly("lane_corridors",
    &RoadCorridor::GetUniqueLaneCorridors)
  .def(py::pickle(
    [](const RoadCorridor& rc) -> py::tuple {
      return py::make_tuple(rc.GetRoads(),
                            rc.GetUniqueLaneCorridors(),
                            rc.GetLaneCorridorMap(),
                            rc.GetPolygon());
    },
    [](const py::tuple &t) {
      if (t.size() != 4)
        throw std::runtime_error("Invalid RoadCorridor state!");
      RoadCorridor rc;
      rc.SetRoads(t[0].cast<Roads>());
      rc.SetUniqueLaneCorridors(
        t[1].cast<std::vector<LaneCorridorPtr>>());
      rc.SetLaneCorridorMap(
        t[2].cast<std::map<LaneId, LaneCorridorPtr>>());
      rc.SetPolygon(t[3].cast<Polygon>());
      return rc;
    }));

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

py::class_<Lane,
           std::shared_ptr<Lane>>(m, "Lane")
  .def(py::init<XodrLanePtr>())
  .def_property_readonly("center_line", &Lane::GetCenterLine)
  .def_property_readonly("right_lane", &Lane::GetRightLane)
  .def_property_readonly("left_lane", &Lane::GetLeftLane)
  .def_property_readonly("left_boundary", &Lane::GetLeftBoundary)
  .def_property_readonly("right_boundary", &Lane::GetRightBoundary)
  .def_property_readonly("next_lane", &Lane::GetNextLane)
  .def_property_readonly("lane_id", &Lane::GetId)
  .def_property_readonly("type", &Lane::GetLaneType)
  .def_property_readonly("driving_direction", &Lane::GetDrivingDirection)
  .def_property_readonly("lane_position", &Lane::GetLanePosition)
  .def_property_readonly("polygon", &Lane::GetPolygon);

py::class_<Road,
           std::shared_ptr<Road>>(m, "Road")
  .def(py::init<XodrRoadPtr>())
  .def_property_readonly("next_road", &Road::GetNextRoad)
  .def_property_readonly("road_id", &Road::GetId)
  .def_property_readonly("lanes", &Road::GetLanes)
  .def("GetLane", &Road::GetLane);
}
