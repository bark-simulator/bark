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
  def test_two_roads_one_lane(self):
    xodr_map = MakeXodrMapOneRoadTwoLanes()

    # World Definition
    params = ParameterServer()
    world = World(params)

    map_interface = MapInterface()
    map_interface.SetOpenDriveMap(xodr_map)
    world.SetMap(map_interface)
    open_drive_map = world.map.GetOpenDriveMap()
    viewer = MPViewer(params=params,
                      use_world_bounds=True)

    # Draw map
    viewer.drawWorld(world)
    viewer.show(block=False)

    # Generate RoadCorridor
    roads = [100] 
    driving_direction = XodrDrivingDirection.forward
    map_interface.GenerateRoadCorridor(roads, driving_direction)
    road_corridor = map_interface.GetRoadCorridor(roads, driving_direction)

    # Assert road corridor
    
    # Assert: 1 road
    self.assertEqual(len(road_corridor.roads), 1)
    
    # Assert: road1: 2 lanes
    self.assertEqual(len(road_corridor.GetRoad(roads[0]).lanes), 3)

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
    viewer.show(block=True)

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

    # Assert: left and right lanes
    # TODO(hart): this has changed due to using the driving direction
    #             these roads are forward and backward
    # self.assertEqual(road_corridor.GetRoad(0).GetLane(2).right_lane.lane_id, 3)
    # self.assertEqual(road_corridor.GetRoad(0).GetLane(3).left_lane.lane_id, 2)
    # self.assertEqual(road_corridor.GetRoad(2).GetLane(7).right_lane.lane_id, 8)
    # self.assertEqual(road_corridor.GetRoad(2).GetLane(8).left_lane.lane_id, 7)
    
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

  @unittest.skip
  def test_road_corridor_highway(self):
    xodr_parser = XodrParser("modules/runtime/tests/data/city_highway_straight.xodr")

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
    roads = [16] 
    driving_direction = XodrDrivingDirection.forward
    map_interface.GenerateRoadCorridor(roads, driving_direction)
    road_corridor = map_interface.GetRoadCorridor(roads, driving_direction)

    # Assert road corridor
    
    # Assert: 3 roads
    # self.assertEqual(len(road_corridor.roads), 3)
    
    # # Assert: road1: 2 lanes, road2: 1 lane, road3: 1 lane
    # self.assertEqual(len(road_corridor.GetRoad(0).lanes), 3)
    # self.assertEqual(len(road_corridor.GetRoad(1).lanes), 2)
    # self.assertEqual(len(road_corridor.GetRoad(2).lanes), 3)

    # # Assert: left and right lanes
    # self.assertEqual(road_corridor.GetRoad(0).GetLane(2).right_lane.lane_id, 3)
    # self.assertEqual(road_corridor.GetRoad(0).GetLane(3).left_lane.lane_id, 2)
    # self.assertEqual(road_corridor.GetRoad(2).GetLane(7).right_lane.lane_id, 8)
    # self.assertEqual(road_corridor.GetRoad(2).GetLane(8).left_lane.lane_id, 7)
    
    # # Assert: next road
    # self.assertEqual(road_corridor.GetRoad(0).next_road.road_id, 1)
    # self.assertEqual(road_corridor.GetRoad(1).next_road.road_id, 2)

    # # Assert: lane links
    # self.assertEqual(road_corridor.GetRoad(0).GetLane(3).next_lane.lane_id, 5)
    # self.assertEqual(road_corridor.GetRoad(1).GetLane(5).next_lane.lane_id, 8)

    # # Assert: LaneCorridor
    # self.assertEqual(len(road_corridor.lane_corridors), 3)

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
    viewer.show(block=True)

  @unittest.skip
  def test_road_corridor_merging(self):
    xodr_parser = XodrParser("modules/runtime/tests/data/DR_DEU_Merging_MT.xodr")

    # World Definition
    params = ParameterServer()
    world = World(params)

    map_interface = MapInterface()
    map_interface.SetOpenDriveMap(xodr_parser.map)
    world.SetMap(map_interface)
    open_drive_map = world.map.GetOpenDriveMap()
    viewer = MPViewer(params=params,
                      use_world_bounds=True)

    # # Draw map
    viewer.drawWorld(world)
    viewer.show(block=False)

    # Generate RoadCorridor
    roads = [0, 1] 
    driving_direction = XodrDrivingDirection.forward
    map_interface.GenerateRoadCorridor(roads, driving_direction)
    road_corridor = map_interface.GetRoadCorridor(roads, driving_direction)

    colors = ["blue", "red", "green", "yellow"]
    count = 0
    for lane_corridor in road_corridor.lane_corridors:
      viewer.drawPolygon2d(lane_corridor.polygon, color=colors[count], alpha=0.5)
      viewer.drawLine2d(lane_corridor.left_boundary, color="red")
      viewer.drawLine2d(lane_corridor.right_boundary, color="blue")
      viewer.drawLine2d(lane_corridor.center_line, color="black")
      viewer.show(block=False)
      plt.pause(2.)
      count += 1
    viewer.show(block=True)

  @unittest.skip
  def test_road_corridor_intersection(self):
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

    colors = ["blue", "red", "green", "yellow"]
    count = 0

    for road_id, road in road_corridor.roads.items():
      for lane_id, lane in road.lanes.items():
        print(road_id, lane_id, lane.driving_direction)
    for lane_corridor in road_corridor.lane_corridors:
      viewer.drawPolygon2d(lane_corridor.polygon, color=colors[count], alpha=0.5)
      viewer.drawLine2d(lane_corridor.left_boundary, color="red")
      viewer.drawLine2d(lane_corridor.right_boundary, color="blue")
      viewer.drawLine2d(lane_corridor.center_line, color="black")
      viewer.show(block=False)
      plt.pause(0.5)
      count += 1
    viewer.show(block=True)

 
if __name__ == '__main__':
  unittest.main()
