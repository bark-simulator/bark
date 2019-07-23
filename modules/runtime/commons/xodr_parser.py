# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

from lxml import etree
import pprint
from bark.world.opendrive import *
from bark.world.map import *
from bark.geometry import *


class XodrParser(object):
    def __init__(self, file_name):
        self.xodr = self.load_xodr(file_name)
        self.python_map = {}
        self.parse_xml(self.xodr)
        self.map = OpenDriveMap()
        self.convert_to_map(self.python_map)
        self.roadgraph = Roadgraph()
        self.roadgraph.Generate(self.map)

    def load_file(self, file_name):
        return open(file_name, 'r')

    def load_xodr(self, file_name):
        """Function that loads XODR and returns root node of

    Arguments:
      file_name {[string]} -- [File name of XODR]

    Raises:
      TypeError -- [If the file could be parsed]

    Returns:
      [etree] -- [Returns the root node of the etree]
    """
        with self.load_file(file_name) as xodr_file:
            parsed_xodr_file = etree.parse(xodr_file).getroot()
            if not etree.iselement(parsed_xodr_file):
                raise TypeError("Not a valid XODR format.")
            return parsed_xodr_file

    def parse_lane_link(self, lane_link):
        new_link = {}
        if lane_link.find("predecessor") is not None:
            new_link["predecessor"] = lane_link.find("predecessor").get("id")
        if lane_link.find("successor") is not None:
            new_link["successor"] = lane_link.find("successor").get("id")
        return new_link

    def parse_lane_road_mark(self, road_mark):
        new_road_mark = {}
        new_road_mark["s_offset"] = road_mark.get("sOffset")
        new_road_mark["type"] = RoadMarkType.__members__[str(road_mark.get("type"))] # assign enum type # road_mark.get("type")
        new_road_mark["weight"] = road_mark.get("weight")
        new_road_mark["color"] = RoadMarkColor.__members__[str(road_mark.get("color"))] # assign enum type # road_mark.get("type")
        new_road_mark["width"] = road_mark.get("width")
        return new_road_mark

    def parse_lane_width(self, lane_width):
        new_lane_width = {}
        new_lane_width["s_offset"] = lane_width.get("sOffset")
        new_lane_width["a"] = lane_width.get("a")
        new_lane_width["b"] = lane_width.get("b")
        new_lane_width["c"] = lane_width.get("c")
        new_lane_width["d"] = lane_width.get("d")
        return new_lane_width

    def zero_lane_width(self):
        # TODO: do we really need this?
        new_lane_width = {}
        new_lane_width["s_offset"] = 0.0
        new_lane_width["a"] = 0.0
        new_lane_width["b"] = 0.0
        new_lane_width["c"] = 0.0
        new_lane_width["d"] = 0.0
        return new_lane_width


    def parse_lane(self, lanes, lane_section):
        # previous or next polynoamial
        lane_dict = {}
        for lane in lanes:
            lane_dict[int(lane.get("id"))] = lane

        for id, lane in lane_dict.items():
            if str(lane.get("type")) not in ["driving", "border", "sidewalk"]: #LaneType.__members__.keys(): # skip if lane type currently not supported
                continue
            new_lane = {}
            new_lane["id"] = id
            new_lane["type"] = LaneType.__members__[str(lane.get("type"))] # assign enum type
            new_lane["level"] = lane.get("level")
            if lane.find("link") is not None:
                new_lane["link"] = self.parse_lane_link(lane.find("link"))
            if lane.find("roadMark") is not None:
                new_lane["road_mark"] = self.parse_lane_road_mark(lane.find("roadMark"))
            if lane.find("width") is not None:
                    new_lane["width"] = self.parse_lane_width(lane.find("width"))
            else:
                if int(id) == 0:
                    new_lane["width"] = self.zero_lane_width()

            lane_section["lanes"].append(new_lane)
        return lane_section

    def parse_header(self, header):
        new_header = {}
        new_header["north"] = header.get("north")
        new_header["south"] = header.get("south")
        new_header["east"] = header.get("east")
        new_header["west"] = header.get("west")
        self.python_map["header"] = new_header

    def parse_lane_sections(self, lane_sections, road):
        road["lane_sections"] = []
        for lane_section in lane_sections:
            new_lane_section = {}
            new_lane_section["s"] = lane_section.get("s")
            lane_keys = ['left', 'center', 'right']
            new_lane_section["lanes"] = []
            for key in lane_keys:
                if lane_section.find(key) is not None:
                    for lane_group in lane_section.findall(key):
                        new_lane_section = self.parse_lane(
                            lane_group.findall('lane'), new_lane_section)
            road["lane_sections"].append(new_lane_section)
        return road

    def parse_plan_view(self, plan_view, road):
        geometries = plan_view.findall("geometry")
        new_plan_view = {}
        new_plan_view["geometries"] = []
        for geometry in geometries:
            new_geometry = {}
            new_geometry["s"] = geometry.get("s")
            new_geometry["x"] = geometry.get("x")
            new_geometry["y"] = geometry.get("y")
            new_geometry["hdg"] = geometry.get("hdg")
            new_geometry["length"] = geometry.get("length")
            if geometry.find("line") is not None:
                new_line = {"type": "line"}
                new_geometry["geometry"] = new_line
            if geometry.find("spiral") is not None:
                new_spiral = {"type": "spiral"}
                new_spiral["curv_start"] = geometry.find("spiral").get(
                    "curvStart")
                new_spiral["curv_end"] = geometry.find("spiral").get("curvEnd")
                new_geometry["geometry"] = new_spiral
            if geometry.find("arc") is not None:
                new_arc = {"type": "arc"}
                new_arc["curvature"] = geometry.find("arc").get("curvature")
                new_geometry["geometry"] = new_arc
            new_plan_view["geometries"].append(new_geometry)
        road["plan_view"] = new_plan_view
        return road

    def parse_road_link_pre_suc(self, element):
        new_pre_suc = {}
        if element.get("elementType") is not None:
            new_pre_suc["element_type"] = element.get("elementType")
        if element.get("elementId") is not None:
            new_pre_suc["element_id"] = element.get("elementId")
        if element.get("contactPoint") is not None:
            new_pre_suc["contact_point"] = element.get("contactPoint")
        return new_pre_suc

    def parse_road_link(self, road_link):
        new_road_link = {}
        if road_link.find("predecessor") is not None:
            new_road_link["predecessor"] = self.parse_road_link_pre_suc(
                road_link.find("predecessor"))
        if road_link.find("successor") is not None:
            new_road_link["successor"] = self.parse_road_link_pre_suc(
                road_link.find("successor"))
        return new_road_link

    def parse_road(self, road):
        new_road = {}
        new_road["id"] = road.get("id")
        new_road["length"] = road.get("length")
        new_road["junction"] = road.get("junction")
        new_road["name"] = road.get("name")
        lanes = road.find("lanes")
        lane_sections = lanes.findall("laneSection")
        new_road = self.parse_lane_sections(lane_sections, new_road)
        new_road = self.parse_plan_view(road.find("planView"), new_road)
        new_road["link"] = self.parse_road_link(road.find("link"))
        self.python_map["roads"].append(new_road)

    def parse_junction_links(self, connection):
        new_links = []
        for link in connection.findall("laneLink"):
            new_link = {}
            new_link["from"] = link.get("from")
            new_link["to"] = link.get("to")
            new_links.append(new_link)
        return new_links

    def parse_junctions(self, junctions):
        for junction in junctions:
            new_junction = {}
            new_junction["id"] = junction.get("id")
            new_junction["connections"] = []
            for connection in junction.findall("connection"):
                new_connection = {}
                new_connection["id"] = connection.get("id")
                new_connection["incoming_road"] = connection.get(
                    "incomingRoad")
                new_connection["connecting_road"] = connection.get(
                    "connectingRoad")
                new_connection["contact_point"] = connection.get(
                    "contactPoint")
                new_connection["lane_links"] = self.parse_junction_links(
                    connection)
                new_junction["connections"].append(new_connection)
            self.python_map["junctions"].append(new_junction)

    def parse_xml(self, xodr_obj):
        """Imports the XODR file to python

    Arguments:
      xodr_obj {etree} -- containing map information

    Returns:
      python dict -- containing all neccessary map information
    """
        self.parse_header(xodr_obj.find("header"))
        self.python_map["roads"] = []
        self.python_map["junctions"] = []
        for road in xodr_obj.findall("road"):
            self.parse_road(road)
        self.parse_junctions(xodr_obj.findall("junction"))

    def print_python_map(self):
        pp = pprint.PrettyPrinter(indent=2)
        pp.pprint(self.python_map)

    def create_cpp_plan_view(self, plan_view):
        new_plan_view = PlanView()
        # create plan view..
        for geometry in plan_view["geometries"]:
            starting_point = Point2d(
                float(geometry["x"]), float(geometry["y"]))
            if geometry["geometry"]["type"] == "line":
                new_plan_view.add_line(starting_point, float(geometry["hdg"]),
                                       float(geometry["length"]))
            if geometry["geometry"]["type"] == "arc":
                new_plan_view.add_arc(starting_point, float(geometry["hdg"]),
                                      float(geometry["length"]),
                                      float(geometry["geometry"]["curvature"]),
                                      0.25) # TODO: s_inc
            if geometry["geometry"]["type"] == "spiral":
                new_plan_view.add_spiral(
                    starting_point, float(geometry["hdg"]),
                    float(geometry["length"]),
                    float(geometry["geometry"]["curv_start"]),
                    float(geometry["geometry"]["curv_end"]), 2) # TODO: s_inc
        return new_plan_view

    def create_cpp_road_link(self, link):
        # TODO(hart): insert road_link
        new_link = RoadLink()
        try:
            new_pre_info = RoadLinkInfo()
            new_pre_info.id = int(link["predecessor"]["element_id"])
            new_pre_info.type = link["predecessor"]["element_type"]
            new_link.predecessor = new_pre_info
        except:
            pass

        try:
            new_suc_info = RoadLinkInfo()
            new_suc_info.id = int(link["successor"]["element_id"])
            new_suc_info.type = link["successor"]["element_type"]
            new_link.successor = new_suc_info
        except:
            pass
        return new_link

    def create_cpp_road(self, road):
        new_road = Road()
        new_road.id = int(road["id"])
        new_road.name = road["name"]
        new_road.plan_view = self.create_cpp_plan_view(road["plan_view"])
        new_road = self.create_cpp_lane_section(new_road, road)
        new_road.link = self.create_cpp_road_link(road["link"])

        return new_road

    def create_lane_link(self, link):
        new_link = LaneLink()

        try:
            new_link.from_position = int(link["predecessor"])
        except:
            print("No LaneLink.predecessor")
        try:
            new_link.to_position = int(link["successor"])
        except:
            print("No LaneLink.successor")
            
        return new_link

    def create_cpp_lane(self, new_lane_section, new_road, lane, s_start,
                        s_end, reference_line):
        try:
            lane_widths = []
            a = float(lane["width"]["a"])
            b = float(lane["width"]["b"])
            c = float(lane["width"]["c"])
            d = float(lane["width"]["d"])
            offset = LaneOffset(a, b, c, d)
            lane_width = LaneWidth(s_start, s_end, offset)

            # TODO (@hart): make sampling flexible           
            new_lane = Lane.create_lane_from_lane_width(int(lane["id"]), reference_line, lane_width, 1.0)

            new_lane.lane_type = lane["type"]
            new_lane.link = self.create_lane_link(lane["link"])

            # not every lane contains a road-mark
            if "road_mark" in lane and lane['road_mark']['type'] != RoadMarkType.none:
                rm = RoadMark()
                rm.type = lane['road_mark']['type']
                rm.color = lane['road_mark']['color']
                rm.width = float(lane['road_mark']['width'])
                new_lane.road_mark = rm
            
            new_lane_section.add_lane(new_lane)

        except:
            raise ValueError("Something went wrong with creating the lane.")
        return new_lane_section

    def sortedIndices(self, lst):
        return sorted(range(len(lst)), key = lambda x:(abs(lst[x])))

    def create_cpp_lane_section(self, new_road, road):
        for lane_section in road["lane_sections"]:
            new_lane_section = LaneSection(float(lane_section["s"]))
            # sort lanes
            #for idx_iterator in range(len(lane_section["lanes"])):
            #    if lane_section["lanes"][idx_iterator]['id'] in list_id_read_in_lanes:

            unordered_id_list = [l['id'] for l in lane_section["lanes"]]
            indices = self.sortedIndices(unordered_id_list)

            #for idx, lane in enumerate(lane_section["lanes"]):
            for idx_iterator in indices:
                lane = lane_section["lanes"][idx_iterator]
                if lane['id'] == 0:
                    # plan view
                    new_lane_section = self.create_cpp_lane(new_lane_section, new_road, lane, 0.0, float(road["length"]), new_road.plan_view.get_reference_line())
                elif lane['id'] == -1 or lane['id'] == 1:
                    # use plan view for offset calculation
                    new_lane_section = self.create_cpp_lane(new_lane_section, new_road, lane, 0.0, float(road["length"]), new_road.plan_view.get_reference_line())
                else:
                    # use previous line for offset calculation
                    #temp_lanes = new_lane_section.get_lanes()

                    if lane['id'] > 0:
                        previous_line = new_lane_section.get_lane_by_position(lane['id']-1).line
                        new_lane_section = self.create_cpp_lane(new_lane_section, new_road, lane, 0.0, previous_line.length(), previous_line)                                
                    elif lane['id'] < 0:
                        previous_line = new_lane_section.get_lane_by_position(lane['id']+1).line
                        new_lane_section = self.create_cpp_lane(new_lane_section, new_road, lane, 0.0, previous_line.length(), previous_line)
                    else:
                        print("Calculating previous lane does not work well.")


            new_road.add_lane_section(new_lane_section)
        return new_road

    def get_map_element(self, key, id):
        for x in self.python_map[key]:
            if x["id"] == id:
                return x

    def create_cpp_junction(self, junction):
        new_junction = Junction("", int(junction["id"]))
        for connection in junction["connections"]:
            new_connection = Connection()
            new_connection.incoming_road = int(connection["incoming_road"])
            new_connection.connecting_road = int(connection["connecting_road"])
            new_connection.id = int(connection["id"])
            # TODO(hart): contact point
            for lane_link in connection["lane_links"]:
                new_lane_link = LaneLink()
                new_lane_link.from_position = int(lane_link["from"])
                new_lane_link.to_position = int(lane_link["to"])
                new_connection.add_lane_link(new_lane_link)
            new_junction.add_connection(new_connection)
        return new_junction

    def convert_to_map(self, python_map):
        """Loops through python_map and converts it to a cpp map

    Arguments:
      python_map {dict} -- containing all the map info

    Returns:
      CPP Map -- Map for usage with CPP
    """
        for road in self.python_map["roads"]:
            new_road = self.create_cpp_road(road)
            self.map.add_road(new_road)

        for junction in self.python_map["junctions"]:
            new_junction = self.create_cpp_junction(junction)
            self.map.add_junction(new_junction)
