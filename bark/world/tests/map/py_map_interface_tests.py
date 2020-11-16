# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import unittest
import time
import math
import filecmp
import matplotlib.pyplot as plt
from bark.core.world.agent import Agent
from bark.core.world import World
from bark.core.geometry import Point2d, Polygon2d
from bark.core.geometry import Point2d
from bark.runtime.commons.parameters import ParameterServer
from bark.core.world.opendrive import OpenDriveMap, MakeXodrMapOneRoadTwoLanes, \
    XodrLaneType, MakeXodrMapCurved, XodrDrivingDirection
from bark.core.world.map import MapInterface
from bark.runtime.commons.xodr_parser import XodrParser
from bark.runtime.viewer.matplotlib_viewer import MPViewer
import numpy as np
import os


class EnvironmentTests(unittest.TestCase):
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

        # if this is not here, the second unit test is not executed (maybe parsing takes too long?)
        time.sleep(1)

    
    def test_curved_road(self):
        params = ParameterServer()
        world = World(params)

        xodr_map = MakeXodrMapCurved(50, 0.1)

        map_interface = MapInterface()
        map_interface.SetOpenDriveMap(xodr_map)
        world.SetMap(map_interface)

        roads = [100]
        driving_direction = XodrDrivingDirection.forward
        map_interface.GenerateRoadCorridor(roads, driving_direction)
        road_corr = map_interface.GetRoadCorridor(roads, driving_direction)

        viewer = MPViewer(params=params, use_world_bounds=True)
        viewer.drawWorld(world)
        viewer.drawRoadCorridor(road_corr)
        #viewer.show(block=True)

        # if this is not here, the second unit test is not executed (maybe parsing takes too long?)
        time.sleep(1)

    def test_between_lanes(self):
        xodr_parser = XodrParser(os.path.join(os.path.dirname(
            __file__), "../../../runtime/tests/data/city_highway_straight.xodr"))
        np.set_printoptions(precision=8)
        params = ParameterServer()

        world = World(params)
        map_interface = MapInterface()
        map_interface.SetOpenDriveMap(xodr_parser.map)
        world.SetMap(map_interface)

        # Simple test
        point_close = Point2d(2, -92.55029)
        lane_sw = map_interface.FindLane(point_close)
        self.assertIsNotNone(
            lane_sw, "This point is still in the left lane! XodrLane boundary is 5114.683")

        switched_lane = False
        lng_coord = -92.55029
        i = 1.817
        lane_sw = map_interface.FindLane(Point2d(i, lng_coord))
        assert lane_sw != None
        prev = lane_sw.lane_id
        prev_i = i
        while (i < 4.817):
            lane_sw = map_interface.FindLane(Point2d(i, lng_coord))
            self.assertIsNotNone(
                lane_sw, "Should always be on at least one lane! Currently at ({}, {})".format(i, lng_coord))
            if prev != lane_sw.lane_id:
                # print(prev)
                # print(prev_i)
                # print(lane_sw.lane_id)
                # print(i)
                self.assertFalse(
                    switched_lane, "XodrLane switch should only happens once!")
                switched_lane = True
            prev_i = i
            prev = lane_sw.lane_id
            i = i + 0.01
        self.assertTrue(
            switched_lane, "Eventually should have switched lanes!")


if __name__ == '__main__':
    unittest.main()
