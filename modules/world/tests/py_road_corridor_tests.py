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


class RoadCorridorTests(unittest.TestCase):
  def test_Crossing8Course(self):
    xodr_parser = XodrParser("modules/runtime/tests/data/road_corridor_test.xodr")

    # World Definition
    params = ParameterServer()
    world = World(params)

    map_interface = MapInterface()
    map_interface.set_open_drive_map(xodr_parser.map)
    world.set_map(map_interface)
    open_drive_map = world.map.get_open_drive_map()
    viewer = MPViewer(params=params,
                      use_world_bounds=True)

    # Draw map
    viewer.drawWorld(world)
    viewer.show(block=False)

    # Generate RoadCorridor
    roads = [0, 1, 2] 
    driving_direction = XodrDrivingDirection.forward
    map_interface.GenerateRoadCorridor(roads, driving_direction)

    road_corridor = map_interface.GetRoadCorridor(roads, driving_direction)

    # Assert road corridor
    
    # Assert: 3 roads
    self.assertEqual(len(road_corridor.roads), 3)
    
    # Assert: road1: 2 lanes, road2: 1 lane, road3: 1 lane
    self.assertEqual(len(road_corridor.get_road(0).lanes), 3)
    self.assertEqual(len(road_corridor.get_road(1).lanes), 2)
    self.assertEqual(len(road_corridor.get_road(2).lanes), 3)

    # Assert: all lanes should have center and boundary lines as well as polygons
    colors = ["blue", "red", "green"]
    count = 0
    for road_id, road in road_corridor.roads.items():
      for lane_id, lane in road.lanes.items():
        viewer.drawLine2d(lane.center_line, color="black")
        viewer.drawLine2d(lane.left_boundary.line, color="red")
        viewer.drawLine2d(lane.right_boundary.line, color="blue")
        # viewer.drawPolygon2d(lane.polygon, color=colors[count], alpha=1.)
        count += 1
        # viewer.drawLine2d(lane.right_boundary)
        viewer.show(block=False)
        plt.pause(1.0)

    
    # Assert: LaneCorridor
    viewer.drawRoadCorridor(road_corridor)
    viewer.show(block=True)

if __name__ == '__main__':
  unittest.main()
