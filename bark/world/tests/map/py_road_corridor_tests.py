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
import itertools
import filecmp
import matplotlib.pyplot as plt
from bark.core.world import World
from bark.core.geometry import *
from bark.runtime.commons.parameters import ParameterServer
from bark.core.world.opendrive import XodrDrivingDirection, MakeXodrMapOneRoadTwoLanes
from bark.core.world.map import MapInterface
from bark.runtime.commons.xodr_parser import XodrParser
from bark.runtime.viewer.matplotlib_viewer import MPViewer
import numpy as np
import os


class RoadCorridorTests(unittest.TestCase):
    @unittest.skip("...")
    def test_road_corridor_forward(self):
        xodr_parser = XodrParser(os.path.join(os.path.dirname(
            __file__), "../../../runtime/tests/data/road_corridor_test.xodr"))

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

    def test_three_way_intersection(self):
        # threeway_intersection
        xodr_parser = XodrParser(os.path.join(os.path.dirname(
            __file__),  "../../../runtime/tests/data/threeway_intersection.xodr"))

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
        xodr_parser = XodrParser(os.path.join(os.path.dirname(
            __file__), "../../../runtime/tests/data/DR_DEU_Merging_MT_v01_shifted.xodr"))

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

        viewer.drawPolygon2d(
            road_corridor.lane_corridors[0].polygon, color="blue", alpha=0.5)
        viewer.drawPolygon2d(
            road_corridor.lane_corridors[1].polygon, color="blue", alpha=0.5)
        viewer.show(block=False)

        self.assertTrue(road_corridor.lane_corridors[0].polygon.Valid())
        self.assertTrue(road_corridor.lane_corridors[1].polygon.Valid())
        self.assertTrue(road_corridor.polygon.Valid())

    def test_dr_deu_merging_centered(self):
        # threeway_intersection
        xodr_parser = XodrParser(os.path.join(os.path.dirname(
            __file__), "../../../runtime/tests/data/DR_DEU_Merging_MT_v01_centered.xodr"))

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

        viewer.drawPolygon2d(
            road_corridor.lane_corridors[0].polygon, color="blue", alpha=0.5)
        viewer.drawPolygon2d(
            road_corridor.lane_corridors[1].polygon, color="blue", alpha=0.5)
        viewer.show(block=False)

        self.assertTrue(road_corridor.lane_corridors[0].polygon.Valid())
        self.assertTrue(road_corridor.lane_corridors[1].polygon.Valid())
        self.assertTrue(road_corridor.polygon.Valid())

        tol = 0.2
        center_line_array = road_corridor.lane_corridors[0].center_line.ToArray()
        left_boundary_array = road_corridor.lane_corridors[0].left_boundary.ToArray()
        right_boundary_array = road_corridor.lane_corridors[0].right_boundary.ToArray()

        # beginning of left lane
        self.assertAlmostEquals(center_line_array[0, 0], 106.4, 1)
        self.assertAlmostEquals(center_line_array[0, 1], 103.47, 1)
        self.assertAlmostEquals(Distance(Point2d(left_boundary_array[0, 0], left_boundary_array[0, 1]), Point2d(right_boundary_array[0, 0], right_boundary_array[0, 1])), 2.8, 1)

        # end of left lane
        self.assertAlmostEquals(center_line_array[-1, 0], -18.15, 1)
        self.assertAlmostEquals(center_line_array[-1, 1], 109.0, 1)
        self.assertAlmostEquals(Distance(Point2d(left_boundary_array[-1, 0], left_boundary_array[-1, 1]), Point2d(right_boundary_array[-1, 0], right_boundary_array[-1, 1])), 2.63, 1)

    def test_dr_chn_merging(self):
        # threeway_intersection
        xodr_parser = XodrParser(os.path.join(os.path.dirname(
            __file__), "../../../runtime/tests/data/DR_CHN_Merging_ZS_partial_v02.xodr"))

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

        viewer.drawPolygon2d(
            road_corridor.lane_corridors[0].polygon, color="blue", alpha=0.5)
        viewer.drawPolygon2d(
            road_corridor.lane_corridors[1].polygon, color="blue", alpha=0.5)
        viewer.drawPolygon2d(
            road_corridor.lane_corridors[2].polygon, color="blue", alpha=0.5)
        viewer.show(block=False)

        self.assertTrue(road_corridor.lane_corridors[0].polygon.Valid())
        self.assertTrue(road_corridor.lane_corridors[1].polygon.Valid())
        self.assertTrue(road_corridor.lane_corridors[2].polygon.Valid())
        self.assertTrue(road_corridor.polygon.Valid())


if __name__ == '__main__':
    unittest.main()
