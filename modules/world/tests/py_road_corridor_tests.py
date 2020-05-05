# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import unittest
import time
import math
import filecmp
import matplotlib.pyplot as plt
from bark.world import World
from modules.runtime.commons.parameters import ParameterServer
from bark.world.opendrive import XodrDrivingDirection, MakeXodrMapOneRoadTwoLanes
from bark.world.map import MapInterface
from modules.runtime.commons.xodr_parser import XodrParser
from modules.runtime.viewer.matplotlib_viewer import MPViewer
import numpy as np


class RoadCorridorTests(unittest.TestCase):
  def test_road_corridor_forward(self):
    xodr_parser = XodrParser("modules/runtime/tests/data/road_corridor_test.xodr")

    # World Definition
    params = ParameterServer()
    world = World(params)

    map_interface = MapInterface()
    map_interface.SetOpenDriveMap(xodr_parser.map)
    world.SetMap(map_interface)
    open_drive_map = world.map.GetOpenDriveMap()
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
    self.assertEqual(len(road_corridor.GetRoad(0).lanes), 3)
    self.assertEqual(len(road_corridor.GetRoad(1).lanes), 2)
    self.assertEqual(len(road_corridor.GetRoad(2).lanes), 3)
    
    # Assert: next road
    self.assertEqual(road_corridor.GetRoad(0).next_road.road_id, 1)
    self.assertEqual(road_corridor.GetRoad(1).next_road.road_id, 2)

    # Assert: lane links
    self.assertEqual(road_corridor.GetRoad(0).GetLane(3).next_lane.lane_id, 5)
    self.assertEqual(road_corridor.GetRoad(1).GetLane(5).next_lane.lane_id, 8)

    # Assert: LaneCorridor
    self.assertEqual(len(road_corridor.lane_corridors), 3)

    colors = ["blue", "red", "green"]
    count = 0
    for lane_corridor in road_corridor.lane_corridors:
      viewer.drawPolygon2d(lane_corridor.polygon, color=colors[count], alpha=0.5)
      viewer.drawLine2d(lane_corridor.left_boundary, color="red")
      viewer.drawLine2d(lane_corridor.right_boundary, color="blue")
      viewer.drawLine2d(lane_corridor.center_line, color="black")
      viewer.show(block=False)
      plt.pause(2.)
      count += 1

 
if __name__ == '__main__':
  unittest.main()
