# Copyright (c) 2020 Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import unittest
from unittest.mock import patch
import time
import math
import itertools
import filecmp
import os
import matplotlib.pyplot as plt
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD:modules/world/tests/py_road_corridor_tests.py
from bark.world import World
from bark.geometry import *
from modules.runtime.commons.parameters import ParameterServer
from bark.world.opendrive import XodrDrivingDirection, MakeXodrMapOneRoadTwoLanes
from bark.world.map import MapInterface
from modules.runtime.commons.xodr_parser import XodrParser
from modules.runtime.viewer.matplotlib_viewer import MPViewer
=======
from bark.core.core.world import World
=======
from bark.pybark.core.world import World
>>>>>>> Wrapped C++ bark in pybark
=======
from bark.core.world import World
>>>>>>> Set python import path to directly include bark.core
from bark.runtime.commons.parameters import ParameterServer
from bark.core.world.opendrive import XodrDrivingDirection, MakeXodrMapOneRoadTwoLanes
from bark.core.world.map import MapInterface
<<<<<<< HEAD
from bark.runtime.commons.xodr_parser import XodrParser
from bark.runtime.viewer.matplotlib_viewer import MPViewer
>>>>>>> Package Restructuring:bark/world/tests/py_road_corridor_tests.py
=======
from bark.runtime.commons import XodrParser
from bark.runtime.viewer import MPViewer
>>>>>>> add all tests as dependency to pip_packange and fixed tests for setup.py test
import numpy as np


class RoadCorridorTests(unittest.TestCase):
    @unittest.skip("...")
    def test_road_corridor_forward(self):
        xodr_parser = XodrParser(
            "modules/runtime/tests/data/road_corridor_test.xodr")

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
        self.assertEqual(road_corridor.GetRoad(
            0).GetLane(3).next_lane.lane_id, 5)
        self.assertEqual(road_corridor.GetRoad(
            1).GetLane(5).next_lane.lane_id, 8)

        # Assert: LaneCorridor
        self.assertEqual(len(road_corridor.lane_corridors), 3)

        colors = ["blue", "red", "green"]
        count = 0
        for lane_corridor in road_corridor.lane_corridors:
            viewer.drawPolygon2d(lane_corridor.polygon,
                                 color=colors[count], alpha=0.5)
            viewer.drawLine2d(lane_corridor.left_boundary, color="red")
            viewer.drawLine2d(lane_corridor.right_boundary, color="blue")
            viewer.drawLine2d(lane_corridor.center_line, color="black")
            viewer.show(block=False)
            plt.pause(2.)
            count += 1
    
<<<<<<< HEAD
    def test_three_way_intersection(self):
        # threeway_intersection
        xodr_parser = XodrParser(
            "modules/runtime/tests/data/threeway_intersection.xodr")

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

        # check few corridors
        def GenerateRoadCorridor(map_interface, comb):
            (start_p, end_p) = comb
            polygon = Polygon2d(
                [0, 0, 0], [Point2d(-1, -1), Point2d(-1, 1), Point2d(1, 1), Point2d(1, -1)])
            start_polygon = polygon.Translate(start_p)
            goal_polygon = polygon.Translate(end_p)
            rc = map_interface.GenerateRoadCorridor(start_p, goal_polygon)
            return rc

        # assert road ids
        rc = GenerateRoadCorridor(map_interface, comb_all[0])
        self.assertEqual(rc.road_ids, [0, 11, 1])
        self.assertEqual(len(rc.lane_corridors), 3)
        self.assertTrue(rc.polygon.Valid())
        rc = GenerateRoadCorridor(map_interface, comb_all[1])
        self.assertEqual(rc.road_ids, [0, 5, 2])
        self.assertEqual(len(rc.lane_corridors), 3)
        self.assertTrue(rc.polygon.Valid())
        rc = GenerateRoadCorridor(map_interface, comb_all[2])
        self.assertEqual(rc.road_ids, [1, 10, 0])
        self.assertEqual(len(rc.lane_corridors), 3)
        self.assertTrue(rc.polygon.Valid())
        rc = GenerateRoadCorridor(map_interface, comb_all[3])
        self.assertEqual(rc.road_ids, [2, 6, 1])
        self.assertEqual(len(rc.lane_corridors), 3)
        self.assertTrue(rc.polygon.Valid())
        rc = GenerateRoadCorridor(map_interface, comb_all[4])
        self.assertEqual(rc.road_ids, [2, 4, 0])
        self.assertEqual(len(rc.lane_corridors), 3)
        self.assertTrue(rc.polygon.Valid())


    def test_dr_deu_merging(self):
        # threeway_intersection
        xodr_parser = XodrParser(
            "modules/runtime/tests/data/DR_DEU_Merging_MT_v01_shifted.xodr")

        # World Definition
        params = ParameterServer()
        world = World(params)

        map_interface = MapInterface()
        map_interface.SetOpenDriveMap(xodr_parser.map)
        world.SetMap(map_interface)

        roads = [0, 1]
        driving_direction = XodrDrivingDirection.forward
        map_interface.GenerateRoadCorridor(roads, driving_direction)
        road_corridor = map_interface.GetRoadCorridor(roads, driving_direction)

        # Draw map
        viewer = MPViewer(params=params, use_world_bounds=True)
        viewer.drawWorld(world)

        viewer.drawPolygon2d(road_corridor.lane_corridors[0].polygon, color = "blue", alpha = 0.5)
        viewer.drawPolygon2d(road_corridor.lane_corridors[1].polygon, color = "blue", alpha = 0.5)
        viewer.show(block=False)

        self.assertTrue(road_corridor.lane_corridors[0].polygon.Valid())
        self.assertTrue(road_corridor.lane_corridors[1].polygon.Valid())
        self.assertTrue(road_corridor.polygon.Valid())

    def test_dr_chn_merging(self):
        # threeway_intersection
        xodr_parser = XodrParser(
            "modules/runtime/tests/data/DR_CHN_Merging_ZS_partial_v02.xodr")

        # World Definition
        params = ParameterServer()
        world = World(params)

        map_interface = MapInterface()
        map_interface.SetOpenDriveMap(xodr_parser.map)
        world.SetMap(map_interface)

        roads = [0, 1]
        driving_direction = XodrDrivingDirection.forward
        map_interface.GenerateRoadCorridor(roads, driving_direction)
        road_corridor = map_interface.GetRoadCorridor(roads, driving_direction)

        # Draw map
        viewer = MPViewer(params=params, use_world_bounds=True)
        viewer.drawWorld(world)

        viewer.drawPolygon2d(road_corridor.lane_corridors[0].polygon, color = "blue", alpha = 0.5)
        viewer.drawPolygon2d(road_corridor.lane_corridors[1].polygon, color = "blue", alpha = 0.5)
        viewer.drawPolygon2d(road_corridor.lane_corridors[2].polygon, color = "blue", alpha = 0.5)
        viewer.show(block=False)

        self.assertTrue(road_corridor.lane_corridors[0].polygon.Valid())
        self.assertTrue(road_corridor.lane_corridors[1].polygon.Valid())
        self.assertTrue(road_corridor.lane_corridors[2].polygon.Valid())
        self.assertTrue(road_corridor.polygon.Valid())


=======
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
    
  
  @unittest.skip
  def test_road_corridor_forward(self):
    xodr_parser = XodrParser(os.path.join(os.path.dirname(__file__),"../../runtime/tests/data/road_corridor_test.xodr"))

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
    self.assertEqual(road_corridor.GetRoad(0).GetLane(2).right_lane.lane_id, 3)
    self.assertEqual(road_corridor.GetRoad(0).GetLane(3).left_lane.lane_id, 2)
    self.assertEqual(road_corridor.GetRoad(2).GetLane(7).right_lane.lane_id, 8)
    self.assertEqual(road_corridor.GetRoad(2).GetLane(8).left_lane.lane_id, 7)
    
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
    xodr_parser = XodrParser(os.path.join(os.path.dirname(__file__),"../../runtime/tests/data/city_highway_straight.xodr"))

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
    xodr_parser = XodrParser(os.path.join(os.path.dirname(__file__),"../../runtime/tests/data/DR_DEU_Merging_MT.xodr"))

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
    xodr_parser = XodrParser(os.path.join(os.path.dirname(__file__),"../../runtime/tests/data/road_corridor_test.xodr"))

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

 
>>>>>>> bark library -python_warpper, fix import errors, run bazel tests
if __name__ == '__main__':
    unittest.main()
