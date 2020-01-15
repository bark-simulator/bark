// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include <string>
#include "opendrive.hpp"
#include "modules/world/opendrive/commons.hpp"
#include "modules/world/opendrive/plan_view.hpp"
#include "modules/world/opendrive/road.hpp"
#include "modules/world/opendrive/lane_section.hpp"
#include "modules/world/opendrive/opendrive.hpp"
#include "modules/world/opendrive/odrSpiral.hpp"

using namespace modules::world::opendrive;

void python_opendrive(py::module m) {
  py::class_<PlanView, std::shared_ptr<PlanView>>(m, "PlanView")
      .def(py::init<>())
      .def("add_line", &PlanView::add_line, "Add line to planview")
      .def("add_spiral", &PlanView::add_spiral, "Add spiral to planview")
      .def("add_arc", &PlanView::add_arc, "Add arc to planview")
      .def("get_reference_line", &PlanView::get_reference_line, "Return as numpy array");

  py::class_<XodrRoadLinkInfo>(m, "XodrRoadLinkInfo")
      .def(py::init<>())
      .def_readwrite("id", &XodrRoadLinkInfo::id_)
      .def_readwrite("type", &XodrRoadLinkInfo::type_);

  py::class_<XodrRoadLink>(m, "XodrRoadLink")
      .def(py::init<>())
      .def_property("predecessor", &XodrRoadLink::get_predecessor, &XodrRoadLink::set_predecessor)
      .def_property("successor", &XodrRoadLink::get_successor, &XodrRoadLink::set_successor)
      .def(
          "__repr__",
          [](const XodrRoadLink &l) {
          std::stringstream ss;
          ss << "<bark.XodrRoadLink> XodrRoadLink: ";
          ss << modules::world::opendrive::print(l);
          return ss.str();
      });

  py::enum_<XodrLaneType>(m, "XodrLaneType", py::arithmetic())
      .value("driving", XodrLaneType::DRIVING)
      .value("biking", XodrLaneType::BIKING)
      .value("sidewalk", XodrLaneType::SIDEWALK)
      .value("border", XodrLaneType::BORDER)
      .export_values();
  
  py::enum_<roadmark::XodrRoadMarkType>(m, "XodrRoadMarkType", py::arithmetic())
      .value("none", roadmark::XodrRoadMarkType::NONE)
      .value("solid", roadmark::XodrRoadMarkType::SOLID)
      .value("broken", roadmark::XodrRoadMarkType::BROKEN)
      .export_values();
  
  py::enum_<roadmark::XodrRoadMarkColor>(m, "XodrRoadMarkColor", py::arithmetic())
      .value("standard", roadmark::XodrRoadMarkColor::STANDARD)
      .value("white", roadmark::XodrRoadMarkColor::WHITE)
      .value("yellow", roadmark::XodrRoadMarkColor::YELLOW)
      .export_values();

  py::class_<XodrRoadMark>(m, "XodrRoadMark")
      .def(py::init<>())
      .def_readwrite("type", &XodrRoadMark::type_)
      .def_readwrite("color", &XodrRoadMark::color_)
      .def_readwrite("width", &XodrRoadMark::width_)
      .def(
          "__repr__",
          [](const XodrRoadMark &rm) {
          std::stringstream ss;
          ss << "<bark.XodrRoadMark> XodrRoadMark: ";
          ss << modules::world::opendrive::print(rm);
          return ss.str();
      });

  py::class_<XodrLane, std::shared_ptr<XodrLane>>(m, "XodrLane")
      .def(py::init<>())
      .def(py::init<XodrLanePosition&>())
      .def_property("lane_id", &XodrLane::get_id, &XodrLane::set_id)
      .def_property("lane_position", &XodrLane::get_lane_position, &XodrLane::set_lane_position)
      .def_property("lane_type", &XodrLane::get_lane_type, &XodrLane::set_lane_type)
      .def_property("link", &XodrLane::get_link, &XodrLane::set_link)
      .def_property("line", &XodrLane::get_line, &XodrLane::set_line)
      .def_property("road_mark", &XodrLane::get_road_mark, &XodrLane::set_road_mark)
      .def_property("speed", &XodrLane::get_speed, &XodrLane::set_speed)
      .def("append", &XodrLane::append, "Append lane")
      .def("create_lane_from_lane_width", &create_lane_from_lane_width, "Create lane")
      .def(
          "__repr__",
          [](const XodrLane &l) {
          std::stringstream ss;
          ss << "<bark.XodrLane> XodrLane: ";
          ss << modules::world::opendrive::print(l);
          return ss.str();
      });

  py::class_<XodrLaneOffset>(m, "XodrLaneOffset")
      .def(py::init<float, float, float, float>());

  py::class_<XodrLaneWidth>(m, "XodrLaneWidth")
      .def(py::init<float, float, XodrLaneOffset>());

  py::class_<XodrLaneSection, std::shared_ptr<XodrLaneSection>>(m, "XodrLaneSection")
      .def(py::init<float>())
      .def("add_lane", &XodrLaneSection::add_lane, "Add lane element")
      .def("get_lanes", &XodrLaneSection::get_lanes, "Get all lane elements")
      .def("get_left_lane", &XodrLaneSection::get_left_lane, "Get left lane")
      .def("get_right_lane", &XodrLaneSection::get_right_lane, "Get right lane")
      .def("get_lane_by_position", &XodrLaneSection::get_lane_by_position, "Get lane by lane position")
      .def(
          "__repr__",
          [](const XodrLaneSection &ls) {
          std::stringstream ss;
          ss << "<bark.XodrLaneSection> XodrLaneSection: ";
          ss << modules::world::opendrive::print(ls);
          return ss.str();
      });

  py::class_<XodrRoad, std::shared_ptr<XodrRoad>>(m, "XodrRoad")
      .def(py::init<>())
      .def_property("name", &XodrRoad::get_name, &XodrRoad::set_name)
      .def_property("id", &XodrRoad::get_id, &XodrRoad::set_id)
      .def_property("plan_view", &XodrRoad::get_plan_view, &XodrRoad::set_plan_view)
      .def_property("link", &XodrRoad::get_link, &XodrRoad::set_link)
      .def_property_readonly("lane_sections", &XodrRoad::get_lane_sections)
      .def("add_lane_section", &XodrRoad::add_lane_section, "Add a lane section to road");

  py::class_<OpenDriveMap, std::shared_ptr<OpenDriveMap>>(m, "OpenDriveMap")
      .def(py::init<>())
      .def("add_road", &OpenDriveMap::add_road, "Add road element")
      .def("add_junction", &OpenDriveMap::add_junction, "Add junction element")
      .def("get_road", &OpenDriveMap::get_road, "Get road element")
      .def("get_roads", &OpenDriveMap::get_roads, "Get all roads")
      .def("get_junctions", &OpenDriveMap::get_junctions, "Get all junctions");

  py::class_<XodrLaneLink>(m, "XodrLaneLink")
      .def(py::init<>())
      .def_readwrite("from_position", &XodrLaneLink::from_position)
      .def_readwrite("to_position", &XodrLaneLink::to_position);

  py::class_<Connection>(m, "Connection")
      .def(py::init<>())
      .def_readwrite("id", &Connection::id_)
      .def_readwrite("incoming_road", &Connection::incoming_road_)
      .def_readwrite("connecting_road", &Connection::connecting_road_)
      .def_property_readonly("lane_links", &Connection::get_lane_links)
      .def("add_lane_link", &Connection::add_lane_link, "Add add XodrLaneLink");

  py::class_<Junction, std::shared_ptr<Junction>>(m, "Junction")
      .def_property("id", &Junction::get_id, &Junction::set_id)
      .def("get_connections", &Junction::get_connections)
      .def("add_connection", &Junction::add_connection)
      .def(py::init<std::string, int>());

  m.def("fresnel_cos", &fresnelCos);
  m.def("fresnel_sin", &fresnelSin);

}
