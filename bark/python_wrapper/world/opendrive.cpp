// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "opendrive.hpp"
#include <string>
#include "bark/world/opendrive/commons.hpp"
#include "bark/world/opendrive/lane_section.hpp"
#include "bark/world/opendrive/odrSpiral.hpp"
#include "bark/world/opendrive/opendrive.hpp"
#include "bark/world/opendrive/plan_view.hpp"
#include "bark/world/opendrive/road.hpp"
#include "bark/world/tests/make_test_xodr_map.hpp"

using namespace bark::world::opendrive;

void python_opendrive(py::module m) {
  py::class_<PlanView, std::shared_ptr<PlanView>>(m, "PlanView")
      .def(py::init<>())
      .def("AddLine", &PlanView::AddLine, "Add line to planview")
      .def("AddSpiral", &PlanView::AddSpiral, "Add spiral to planview")
      .def("AddArc", &PlanView::AddArc, "Add arc to planview")
      .def("GetReferenceLine", &PlanView::GetReferenceLine,
           "Return as numpy array")
      .def("ApplyOffsetTransform", &PlanView::ApplyOffsetTransform,
           "Apply offset to planview");

  py::class_<XodrRoadLinkInfo>(m, "XodrRoadLinkInfo")
      .def(py::init<>())
      .def_readwrite("id", &XodrRoadLinkInfo::id_)
      .def_readwrite("type", &XodrRoadLinkInfo::type_);

  py::class_<XodrRoadLink>(m, "XodrRoadLink")
      .def(py::init<>())
      .def_property("predecessor", &XodrRoadLink::GetPredecessor,
                    &XodrRoadLink::SetPredecessor)
      .def_property("successor", &XodrRoadLink::GetSuccessor,
                    &XodrRoadLink::SetSuccessor)
      .def("__repr__", [](const XodrRoadLink& l) {
        std::stringstream ss;
        ss << "<bark.XodrRoadLink> XodrRoadLink: ";
        ss << bark::world::opendrive::print(l);
        return ss.str();
      });

  py::enum_<XodrLaneType>(m, "XodrLaneType", py::arithmetic())
      .value("driving", XodrLaneType::DRIVING)
      .value("biking", XodrLaneType::BIKING)
      .value("sidewalk", XodrLaneType::SIDEWALK)
      .value("border", XodrLaneType::BORDER)
      .export_values();

  py::enum_<XodrDrivingDirection>(m, "XodrDrivingDirection", py::arithmetic())
      .value("forward", XodrDrivingDirection::FORWARD)
      .value("backward", XodrDrivingDirection::BACKWARD)
      .value("both", XodrDrivingDirection::BOTH)
      .export_values();

  py::enum_<roadmark::XodrRoadMarkType>(m, "XodrRoadMarkType", py::arithmetic())
      .value("none", roadmark::XodrRoadMarkType::NONE)
      .value("solid", roadmark::XodrRoadMarkType::SOLID)
      .value("broken", roadmark::XodrRoadMarkType::BROKEN)
      .export_values();

  py::enum_<roadmark::XodrRoadMarkColor>(m, "XodrRoadMarkColor",
                                         py::arithmetic())
      .value("standard", roadmark::XodrRoadMarkColor::STANDARD)
      .value("white", roadmark::XodrRoadMarkColor::WHITE)
      .value("yellow", roadmark::XodrRoadMarkColor::YELLOW)
      .export_values();

  py::class_<XodrRoadMark>(m, "XodrRoadMark")
      .def(py::init<>())
      .def_readwrite("type", &XodrRoadMark::type_)
      .def_readwrite("color", &XodrRoadMark::color_)
      .def_readwrite("width", &XodrRoadMark::width_)
      .def("__repr__", [](const XodrRoadMark& rm) {
        std::stringstream ss;
        ss << "<bark.XodrRoadMark> XodrRoadMark: ";
        ss << bark::world::opendrive::print(rm);
        return ss.str();
      });

  py::class_<XodrLane, std::shared_ptr<XodrLane>>(m, "XodrLane")
      .def(py::init<>())
      .def(py::init<XodrLanePosition&>())
      .def_property("lane_id", &XodrLane::GetId, &XodrLane::SetId)
      .def_property("lane_position", &XodrLane::GetLanePosition,
                    &XodrLane::SetLanePosition)
      .def_property("lane_type", &XodrLane::GetLaneType, &XodrLane::SetLaneType)
      .def_property("driving_direction", &XodrLane::GetDrivingDirection,
                    &XodrLane::SetDrivingDirection)
      .def_property("link", &XodrLane::GetLink, &XodrLane::SetLink)
      .def_property("line", &XodrLane::GetLine, &XodrLane::SetLine)
      .def_property("road_mark", &XodrLane::GetRoad_mark,
                    &XodrLane::SetRoadMark)
      .def_property("speed", &XodrLane::GetSpeed, &XodrLane::SetSpeed)
      .def("append", &XodrLane::append, "Append lane")
      .def("CreateLaneFromLaneWidth", &CreateLaneFromLaneWidth, "Create lane")
      .def("__repr__", [](const XodrLane& l) {
        std::stringstream ss;
        ss << "<bark.XodrLane> XodrLane: ";
        ss << bark::world::opendrive::print(l);
        return ss.str();
      });

  py::class_<XodrLaneOffset>(m, "XodrLaneOffset")
      .def(py::init<float, float, float, float>());

  py::class_<XodrLaneWidth>(m, "XodrLaneWidth")
      .def(py::init<float, float, XodrLaneOffset>());

  py::class_<XodrLaneSection, std::shared_ptr<XodrLaneSection>>(
      m, "XodrLaneSection")
      .def(py::init<float>())
      .def("AddLane", &XodrLaneSection::AddLane, "Add lane element")
      .def("GetLanes", &XodrLaneSection::GetLanes, "Get all lane elements")
      .def("GetLaneByPosition", &XodrLaneSection::GetLaneByPosition,
           "Get lane by lane position")
      .def("__repr__", [](const XodrLaneSection& ls) {
        std::stringstream ss;
        ss << "<bark.XodrLaneSection> XodrLaneSection: ";
        ss << bark::world::opendrive::print(ls);
        return ss.str();
      });

  py::class_<XodrRoad, std::shared_ptr<XodrRoad>>(m, "XodrRoad")
      .def(py::init<>())
      .def_property("name", &XodrRoad::GetName, &XodrRoad::SetName)
      .def_property("id", &XodrRoad::GetId, &XodrRoad::SetId)
      .def_property("plan_view", &XodrRoad::GetPlanView, &XodrRoad::SetPlanView)
      .def_property("link", &XodrRoad::GetLink, &XodrRoad::SetLink)
      .def_property_readonly("lane_sections", &XodrRoad::GetLaneSections)
      .def("AddLaneSection", &XodrRoad::AddLaneSection,
           "Add a lane section to road");

  py::class_<OpenDriveMap, std::shared_ptr<OpenDriveMap>>(m, "OpenDriveMap")
      .def(py::init<>())
      .def("AddRoad", &OpenDriveMap::AddRoad, "Add road element")
      .def("AddJunction", &OpenDriveMap::AddJunction, "Add junction element")
      .def("GetRoad", &OpenDriveMap::GetRoad, "Get road element")
      .def("GetRoads", &OpenDriveMap::GetRoads, "Get all roads")
      .def("GetJunctions", &OpenDriveMap::GetJunctions, "Get all junctions");

  m.def("MakeXodrMapOneRoadTwoLanes",
        &bark::world::tests::MakeXodrMapOneRoadTwoLanes);
  m.def("MakeXodrMapTwoRoadsOneLane",
        &bark::world::tests::MakeXodrMapTwoRoadsOneLane);
  m.def("MakeXodrMapEndingLaneInParallel",
        &bark::world::tests::MakeXodrMapEndingLaneInParallel);

  py::class_<XodrLaneLink>(m, "XodrLaneLink")
      .def(py::init<>())
      .def_readwrite("from_position", &XodrLaneLink::from_position)
      .def_readwrite("to_position", &XodrLaneLink::to_position);

  py::class_<Connection>(m, "Connection")
      .def(py::init<>())
      .def_readwrite("id", &Connection::id_)
      .def_readwrite("incoming_road", &Connection::incoming_road_)
      .def_readwrite("connecting_road", &Connection::connecting_road_)
      .def_property_readonly("lane_links", &Connection::GetLaneLinks)
      .def("AddLaneLink", &Connection::AddLaneLink, "Add add XodrLaneLink");

  py::class_<Junction, std::shared_ptr<Junction>>(m, "Junction")
      .def_property("id", &Junction::GetId, &Junction::SetId)
      .def("GetConnections", &Junction::GetConnections)
      .def("AddConnection", &Junction::AddConnection)
      .def(py::init<std::string, int>());

  m.def("FresnelCos", &FresnelCos);
  m.def("FresnelSin", &FresnelSin);
}
