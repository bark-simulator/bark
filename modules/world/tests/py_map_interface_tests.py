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
        map_interface.SetOpenDriveMap(xodr_parser.map)
        world.SetMap(map_interface)

        start_point = Point2d(0, -11)
        lanes_near_start = map_interface.find_nearest_lanes(start_point, 1)
        assert(len(lanes_near_start) == 1)

        goal_point = Point2d(-191.789, -50.1725)
        lanes_near_goal = map_interface.find_nearest_lanes(goal_point, 1)
        assert(len(lanes_near_goal) == 1)


        time.sleep(2)  # if this is not here, the second unit test is not executed (maybe parsing takes too long?)


    def test_two_roads_one_lane(self):

        params = ParameterServer()
        world = World(params)

        xodr_map = MakeXodrMapOneRoadTwoLanes()

        map_interface = MapInterface()
        map_interface.SetOpenDriveMap(xodr_map)
        world.SetMap(map_interface)

        start_point = Point2d(0, -11)
        lanes_near_start = map_interface.find_nearest_lanes(start_point, 1)
        assert(len(lanes_near_start) == 1)

        goal_point = Point2d(-191.789, -50.1725)
        lanes_near_goal = map_interface.find_nearest_lanes(goal_point, 1)
        assert(len(lanes_near_goal) == 1)

        viewer = MPViewer(params=params, use_world_bounds=True)
        viewer.drawWorld(world)

        time.sleep(2)  # if this is not here, the second unit test is not executed (maybe parsing takes too long?)

    def test_between_lanes(self):
        xodr_parser = XodrParser("modules/runtime/tests/data/city_highway_straight.xodr")
        np.set_printoptions(precision=8)
        params = ParameterServer()

        world = World(params)
        map_interface = MapInterface()
        map_interface.SetOpenDriveMap(xodr_parser.map)
        world.SetMap(map_interface)

        # Simple test
        point_close = Point2d(5112.68262, 5086.44971)
        lane_sw = map_interface.find_lane(point_close)
        self.assertIsNotNone(lane_sw, "This point is still in the left lane! XodrLane boundary is 5112.683")

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
                self.assertFalse(switched_lane, "XodrLane switch should only happens once!")
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
        map_interface.SetOpenDriveMap(xodr_parser.map)
        world.SetMap(map_interface)

        lane_sw = map_interface.find_lane(Point2d(46, 180))
        assert lane_sw.lane_type == XodrLaneType.sidewalk

        lane_rl = map_interface.find_lane(Point2d(52, 130))
        assert lane_rl.lane_type == XodrLaneType.driving

        lane_no_lane = map_interface.find_lane(Point2d(120, 140))
        assert lane_no_lane == None

        xodr_parser = XodrParser("modules/runtime/tests/data/city_highway_straight.xodr")
        np.set_printoptions(precision=8)
        params = ParameterServer()
        world = World(params)

        map_interface = MapInterface()
        map_interface.SetOpenDriveMap(xodr_parser.map)
        world.SetMap(map_interface)
        point = Point2d(5111, 5072)
        viewer = MPViewer(params=params, use_world_bounds=True)
        viewer.drawWorld(world)
        viewer.drawPoint2d(point, 'red', 1.0)
        viewer.show(block=True)
        time.sleep(0.1)
        lane_sw = map_interface.find_lane(point)
        self.assertIsNotNone(lane_sw, "This point is clearly on a lane!")

if __name__ == '__main__':
    unittest.main()
