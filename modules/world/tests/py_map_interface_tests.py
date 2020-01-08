# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import unittest
import time
import math
import filecmp
import matplotlib.pyplot as plt
from bark.world.agent import *
from bark.models.behavior import *
from bark.world import *
from bark.geometry import *
from bark.models.dynamic import *
from bark.models.execution import *
from bark.geometry import *
from bark.geometry.standard_shapes import *
from modules.runtime.commons.parameters import ParameterServer
from bark.world.opendrive import *
from bark.world.map import *
from modules.runtime.commons.xodr_parser import XodrParser
from modules.runtime.viewer.matplotlib_viewer import MPViewer
import numpy as np


class EnvironmentTests(unittest.TestCase):
    def test_Crossing8Course(self):
        xodr_parser = XodrParser("modules/runtime/tests/data/Crossing8Course.xodr")

        params = ParameterServer()
        world = World(params)

        map_interface = MapInterface()
        map_interface.set_open_drive_map(xodr_parser.map)
        world.set_map(map_interface)

        start_point = Point2d(0, -11)
        lanes_near_start = map_interface.find_nearest_lanes(start_point, 1)
        assert(len(lanes_near_start) == 1)

        goal_point = Point2d(-191.789, -50.1725)
        lanes_near_goal = map_interface.find_nearest_lanes(goal_point, 1)
        assert(len(lanes_near_goal) == 1)

        driving_corridor = map_interface.compute_driving_corridor_from_start_to_goal(
            lanes_near_start[0].lane_id, lanes_near_goal[0].lane_id)
        print(driving_corridor)
        for id in driving_corridor.get_lane_ids():
            l = map_interface.get_lane(id[1])
            assert(l.lane_type == LaneType.driving)

        time.sleep(2)  # if this is not here, the second unit test is not executed (maybe parsing takes too long?)

    def test_driving_corridor_adjacency_4way_intersection(self):
        #xodr_parser = XodrParser("modules/runtime/tests/data/urban_road.xodr")
        #xodr_parser = XodrParser("modules/runtime/tests/data/city_highway_straight.xodr")
        xodr_parser = XodrParser("modules/runtime/tests/data/4way_intersection.xodr")

        params = ParameterServer()
        world = World(params)

        map_interface = MapInterface()
        map_interface.set_open_drive_map(xodr_parser.map)
        # xodr_parser.roadgraph.print_graph("/home/esterle/4way_intersection.dot")
        world.set_map(map_interface)

        map_interface.compute_all_driving_corridors()

        all_corridors = map_interface.get_all_corridors()
        c = all_corridors[10]
        right_adj_corridors = map_interface.get_adjacent_corridors_same_direction(c, [151, 168, 0.0])
        assert(len(right_adj_corridors) == 2)

        right_adj_corridors = map_interface.get_adjacent_corridors_same_direction(c, [169, 169, 0.0])
        assert(len(right_adj_corridors) == 1)

        viewer = MPViewer(params=params, use_world_bounds=True)
        viewer.drawWorld(world)
        viewer.drawDrivingCorridor(c)
        if right_adj_corridors:
            for rc in right_adj_corridors:
                viewer.drawDrivingCorridor(rc)

        viewer.show(block=True)
        time.sleep(0.1)

    def test_driving_corridor_splitting_4way_intersection(self):
        #xodr_parser = XodrParser("modules/runtime/tests/data/urban_road.xodr")
        #xodr_parser = XodrParser("modules/runtime/tests/data/city_highway_straight.xodr")
        xodr_parser = XodrParser("modules/runtime/tests/data/4way_intersection.xodr")

        params = ParameterServer()
        world = World(params)

        map_interface = MapInterface()
        map_interface.set_open_drive_map(xodr_parser.map)
        # xodr_parser.roadgraph.print_graph("/home/esterle/4way_intersection.dot")
        world.set_map(map_interface)

        map_interface.compute_all_driving_corridors()

        all_corridors = map_interface.get_all_corridors()

        c = all_corridors[11]

        splittingcorridors = map_interface.get_splitting_corridors(c, [168, 161, 0.0])
        assert(len(splittingcorridors) == 0)

        splittingcorridors = map_interface.get_splitting_corridors(c, [150, 168, 0.0])
        assert(len(splittingcorridors) == 2)

        viewer = MPViewer(params=params, use_world_bounds=True)
        viewer.drawWorld(world)
        viewer.drawDrivingCorridor(c)
        if splittingcorridors:
            for sc in splittingcorridors:
                viewer.drawDrivingCorridor(sc)

        viewer.show(block=True)
        time.sleep(0.1)

    def test_between_lanes(self):
        xodr_parser = XodrParser("modules/runtime/tests/data/city_highway_straight.xodr")
        np.set_printoptions(precision=8)
        params = ParameterServer()

        world = World(params)
        map_interface = MapInterface()
        map_interface.set_open_drive_map(xodr_parser.map)
        world.set_map(map_interface)

        # Simple test
        point_close = Point2d(5112.68262, 5086.44971)
        lane_sw = map_interface.find_lane(point_close)
        self.assertIsNotNone(lane_sw, "This point is still in the left lane! Lane boundary is 5112.683")

        switched_lane = False
        lng_coord = 5086.44971
        i = 5112.0
        lane_sw = map_interface.find_lane(Point2d(i, lng_coord))
        assert lane_sw != None
        prev = lane_sw.lane_id
        prev_i = i
        while (i < 5113.0):
            lane_sw = map_interface.find_lane(Point2d(i, lng_coord))
            self.assertIsNotNone(lane_sw, "Should always be on at least one lane! Currently at ({}, {})".format(i, lng_coord))
            if prev != lane_sw.lane_id:
                # print(prev)
                # print(prev_i)
                # print(lane_sw.lane_id)
                # print(i)
                self.assertFalse(switched_lane, "Lane switch should only happens once!")
                switched_lane = True
            prev_i = i
            prev = lane_sw.lane_id
            i = i + 0.0001
        self.assertTrue(switched_lane, "Eventually should have switched lanes!")

    def test_find_lane(self):

        xodr_parser = XodrParser("modules/runtime/tests/data/urban_road.xodr")

        params = ParameterServer()
        world = World(params)

        map_interface = MapInterface()
        map_interface.set_open_drive_map(xodr_parser.map)
        world.set_map(map_interface)

        lane_sw = map_interface.find_lane(Point2d(46, 180))
        assert lane_sw.lane_type == LaneType.sidewalk

        lane_rl = map_interface.find_lane(Point2d(52, 130))
        assert lane_rl.lane_type == LaneType.driving

        lane_no_lane = map_interface.find_lane(Point2d(120, 140))
        assert lane_no_lane == None

        xodr_parser = XodrParser("modules/runtime/tests/data/city_highway_straight.xodr")
        np.set_printoptions(precision=8)
        params = ParameterServer()
        world = World(params)

        map_interface = MapInterface()
        map_interface.set_open_drive_map(xodr_parser.map)
        world.set_map(map_interface)
        point = Point2d(5111, 5072)
        viewer = MPViewer(params=params, use_world_bounds=True)
        viewer.drawWorld(world)
        polygon = world.map.get_roadgraph().get_lane_polygon_by_id(241)
        polygon2 = world.map.get_roadgraph().get_lane_polygon_by_id(242)
        viewer.drawPolygon2d(polygon, 'blue', 1.0)
        viewer.drawPolygon2d(polygon2, 'green', 1.0)
        viewer.drawPoint2d(point, 'red', 1.0)
        viewer.show(block=True)
        time.sleep(0.1)
        lane_sw = map_interface.find_lane(point)
        self.assertIsNotNone(lane_sw, "This point is clearly on a lane!")

    def test_line_segment_within_driving_corridor(self):

        xodr_parser = XodrParser("modules/runtime/tests/data/urban_road.xodr")

        params = ParameterServer()
        world = World(params)

        map_interface = MapInterface()
        map_interface.set_open_drive_map(xodr_parser.map)
        world.set_map(map_interface)

        map_interface.compute_all_driving_corridors()
        all_corridors = map_interface.get_all_corridors()

        point_sw = Point2d(46, 180)  # this point lies inside the sidewalk left of the road
        point_rl = Point2d(52, 130)  # this point lies in the right lane
        point_outside = Point2d(140, 37)  # this point lies far outside (in lane of type NONE)
        point_no_lane = Point2d(120, 140)  # this point lies far outside, not in any lane

        assert not map_interface.line_segment_inside_corridor(all_corridors[0], point_sw, point_rl)
        assert not map_interface.line_segment_inside_corridor(all_corridors[0], point_sw, point_no_lane)
        assert not map_interface.line_segment_inside_corridor(all_corridors[0], point_outside, point_no_lane)
        assert not map_interface.line_segment_inside_corridor(all_corridors[0], point_sw, point_outside)

    def test_driving_direction(self):

        xodr_parser = XodrParser("modules/runtime/tests/data/urban_road.xodr")

        params = ParameterServer()
        world = World(params)

        map_interface = MapInterface()
        map_interface.set_open_drive_map(xodr_parser.map)
        world.set_map(map_interface)

        point_rl = Point2d(52, 130)  # this point lies in the right lane
        point_ll = Point2d(68, 72)  # this point lies in the left lane
        point_no_lane = Point2d(120, 140)  # this point lies far outside, not in any lane

        assert map_interface.has_correct_driving_direction(point_rl, math.pi/2)
        assert map_interface.has_correct_driving_direction(point_rl, math.pi/2+0.2)
        assert map_interface.has_correct_driving_direction(point_rl, math.pi/2-0.2)
        assert not map_interface.has_correct_driving_direction(point_rl, -math.pi/2)

        assert map_interface.has_correct_driving_direction(point_ll, -math.pi/4)
        assert not map_interface.has_correct_driving_direction(point_ll, -math.pi/4 + math.pi)

        assert not map_interface.has_correct_driving_direction(point_no_lane, 0)
        assert not map_interface.has_correct_driving_direction(point_no_lane, math.pi/2)
        assert not map_interface.has_correct_driving_direction(point_no_lane, math.pi)


if __name__ == '__main__':
    unittest.main()
