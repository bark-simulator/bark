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
from bark.geometry import *
from modules.runtime.commons.parameters import ParameterServer
from bark.world.opendrive import XodrDrivingDirection, MakeXodrMapOneRoadTwoLanes
from bark.world.map import MapInterface
from modules.runtime.commons.xodr_parser import XodrParser
from modules.runtime.viewer.matplotlib_viewer import MPViewer
import numpy as np


class RoadCorridorTests(unittest.TestCase):
  @unittest.skip("...")
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

  def test_three_way_plain(self):
    # three_way_plain
    xodr_parser = XodrParser("modules/runtime/tests/data/three_way_plain.xodr")

    # World Definition
    params = ParameterServer()
    world = World(params)

    map_interface = MapInterface()
    map_interface.SetOpenDriveMap(xodr_parser.map)
    world.SetMap(map_interface)
    open_drive_map = world.map.GetOpenDriveMap()
    viewer = MPViewer(params=params,
                      use_world_bounds=True)
    comb_all = []
    start_point = [Point2d(-30, -2)]
    end_point_list = [Point2d(30, -2), Point2d(-2, -30)]
    comb = list(itertools.product(start_point, end_point_list))
    comb_all = comb_all + comb

    # starting on the right
    start_point = [Point2d(30, 2)]
    end_point_list = [Point2d(-30, 2)]
    comb = list(itertools.product(start_point, end_point_list))
    comb_all = comb_all + comb

    # starting on the bottom
    start_point = [Point2d(2, -30)]
    end_point_list = [Point2d(30, -2), Point2d(-30, 2)]
    comb = list(itertools.product(start_point, end_point_list))
    comb_all = comb_all + comb

    for cnt, (start_p, end_p) in enumerate(comb_all):
      polygon = Polygon2d([0, 0, 0], [Point2d(-1,-1),Point2d(-1,1),Point2d(1,1), Point2d(1,-1)])
      start_polygon = polygon.Translate(start_p)
      goal_polygon = polygon.Translate(end_p)
      rc = map_interface.GenerateRoadCorridor(start_p, goal_polygon)
      if rc:
        roads = rc.roads
        road_ids = list(roads.keys())
        print(road_ids, rc.road_ids)
        
        viewer.drawWorld(world)
        viewer.drawRoadCorridor(rc, "blue")
        viewer.saveFig(output_dir + "/" + "roadcorridor_" + str(cnt) + ".png")
        viewer.show()
        viewer.clear()

        for idx, lane_corridor in enumerate(rc.lane_corridors):
          viewer.drawWorld(world)
          viewer.drawLaneCorridor(lane_corridor, "green")
          viewer.drawLine2d(lane_corridor.left_boundary, color="red")
          viewer.drawLine2d(lane_corridor.right_boundary, color="green")
          viewer.drawLine2d(lane_corridor.center_line, color="green")
          viewer.drawPolygon2d(start_polygon, color="green", facecolor="green", alpha=1.)
          viewer.drawPolygon2d(goal_polygon, color="red", facecolor="red", alpha=1.)
          viewer.saveFig(output_dir + "/" + "roadcorridor_" + str(cnt) + "_with_driving_direction_lancecorridor" + str(idx) + ".png")
          viewer.show()
          viewer.clear()
        
        viewer.show()
        viewer.clear()
if __name__ == '__main__':
  unittest.main()
