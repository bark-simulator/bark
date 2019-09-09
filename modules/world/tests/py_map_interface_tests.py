# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import unittest
import time
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


class EnvironmentTests(unittest.TestCase):
    def test_Crossing8Course(self):
        xodr_parser = XodrParser("modules/runtime/tests/data/Crossing8Course.xodr")

        params = ParameterServer()
        world = World(params)

        map_interface = MapInterface()
        map_interface.set_open_drive_map(xodr_parser.map)
        map_interface.set_roadgraph(xodr_parser.roadgraph)
        world.set_map(map_interface)

        start_point = Point2d(0, -11)
        lanes_near_start = map_interface.find_nearest_lanes(start_point, 1)
        assert(len(lanes_near_start) == 1)

        goal_point = Point2d(-191.789,-50.1725)
        lanes_near_goal = map_interface.find_nearest_lanes(goal_point, 1)
        assert(len(lanes_near_goal) == 1)

        driving_corridor = map_interface.compute_driving_corridor_from_start_to_goal(lanes_near_start[0].lane_id, lanes_near_goal[0].lane_id)
        print(driving_corridor)
        for id in driving_corridor.get_lane_ids():
            l = map_interface.get_lane(id[1])
            assert(l.lane_type == LaneType.driving)

        time.sleep(2) # if this is not here, the second unit test is not executed (maybe parsing takes too long?)

    def test_driving_corridor_adjacency_4way_intersection(self):
        #xodr_parser = XodrParser("modules/runtime/tests/data/urban_road.xodr")
        #xodr_parser = XodrParser("modules/runtime/tests/data/city_highway_straight.xodr")
        xodr_parser = XodrParser("modules/runtime/tests/data/4way_intersection.xodr")

        params = ParameterServer()
        world = World(params)

        map_interface = MapInterface()
        map_interface.set_open_drive_map(xodr_parser.map)
        map_interface.set_roadgraph(xodr_parser.roadgraph)
        #xodr_parser.roadgraph.print_graph("/home/esterle/4way_intersection.dot")
        world.set_map(map_interface)

        map_interface.compute_all_driving_corridors()

        all_corridors = map_interface.get_all_corridors()
        c = all_corridors[10]
        right_adj_corridors = map_interface.get_adjacent_corridors(c, [172, 151, 0.0])
        #assert(len(right_adj_corridors) == 1)

        viewer = MPViewer(params=params)
        viewer.drawWorld(world)
        viewer.drawDrivingCorridor(c)
        if right_adj_corridors:
            for rc in right_adj_corridors:
                viewer.drawDrivingCorridor(rc)
        
        viewer.show(block=True)
        time.sleep(0.1)


if __name__ == '__main__':
    unittest.main()
