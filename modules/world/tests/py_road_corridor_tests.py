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
  def test_road_corridor_forward(self):
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

    # Assert: left and right lanes
    self.assertEqual(road_corridor.get_road(0).get_lane(2).right_lane.lane_id, 3)
    self.assertEqual(road_corridor.get_road(0).get_lane(3).left_lane.lane_id, 2)
    self.assertEqual(road_corridor.get_road(2).get_lane(7).right_lane.lane_id, 8)
    self.assertEqual(road_corridor.get_road(2).get_lane(8).left_lane.lane_id, 7)
    
    # Assert: all lanes should have center and boundary lines as well as polygons
    colors = ["blue", "red", "green"]
    count = 0
    for road_id, road in road_corridor.roads.items():
      if road.next_road:
        print("XodrRoadId: {} to {}.".format(str(road_id),
                                             str(road.next_road.road_id)))
      for lane_id, lane in road.lanes.items():
        # print("Current XodrLaneId: {} to {}.".format(str(lane_id),
        #                                              str(lane.next_lane.lane_id)))
        # try:
        #   print("road_id", road_id, ", left_lane", lane.lane_id, lane.lane_position, lane.left_lane.lane_id, lane.left_lane.lane_position)
        # except:
        #   pass
        # try:
        #   print("road_id", road_id, ", right_lane", lane.lane_id,  lane.lane_position, lane.right_lane.lane_id, lane.right_lane.lane_position)
        # except:
        #   pass
        
        viewer.drawLine2d(lane.center_line, color="black")
        # viewer.drawText(position, text)
        viewer.drawLine2d(lane.left_boundary.line, color="red")
        viewer.drawLine2d(lane.right_boundary.line, color="blue")
        # viewer.drawPolygon2d(lane.polygon, color=colors[count], alpha=1.)
        count += 1
        # viewer.drawLine2d(lane.right_boundary)
        viewer.show(block=False)
        # plt.pause(1.0)

    # Assert: LaneCorridor
    # self.assertEqual(len(road_corridor.lane_corridors), 3)
    viewer.drawRoadCorridor(road_corridor)
    viewer.show(block=True)

if __name__ == '__main__':
  unittest.main()
