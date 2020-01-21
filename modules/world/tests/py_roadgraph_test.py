# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import unittest
from bark.world.map import Roadgraph
from bark.world.opendrive import *
from modules.runtime.commons.xodr_parser import XodrParser


class RoadgraphTests(unittest.TestCase):
    def test_world(self):
        road_graph = Roadgraph()
        lane1 = XodrLane()
        lane2 = XodrLane()
        road_id = 0

        # add vertices and connect edges
        u = road_graph.add_lane(road_id, lane1)
        v = road_graph.add_lane(road_id, lane2)
        road_graph.add_inner_neighbor(u, v)
        road_graph.add_outer_neighbor(u, v)

        vertices = road_graph.get_vertices()
        for vertex_descr in vertices:
            vertex = road_graph.get_vertex(vertex_descr)
            print(vertex.lane_id)

        get_edges = road_graph.get_edges()
        for edge_descr in get_edges:
            edge = road_graph.get_edge(edge_descr)
            print(edge.edge_type)
    
    def test_FindRoadPath_4way_intersection(self):

        xodr_parser = XodrParser("modules/runtime/tests/data/4way_intersection.xodr")
        rg = Roadgraph()
        rg.Generate(xodr_parser.map)

        road_path = rg.FindRoadPath(2, 2)
        self.assertEqual(len(road_path), 1)

        road_path = rg.FindRoadPath(2, 8)
        self.assertEqual(len(road_path), 3)
        self.assertEqual(road_path[0], 2)
        self.assertEqual(road_path[1], 5)
        self.assertEqual(road_path[2], 8)

        road_path = rg.FindRoadPath(2, 3)
        self.assertEqual(len(road_path), 3)
        self.assertEqual(road_path[0], 2)
        self.assertEqual(road_path[1], 10)
        self.assertEqual(road_path[2], 3)

        road_path = rg.FindRoadPath(2, 15)
        self.assertEqual(len(road_path), 3)
        self.assertEqual(road_path[0], 2)
        self.assertEqual(road_path[1], 17)
        self.assertEqual(road_path[2], 15)

        road_path = rg.FindRoadPath(8, 15)
        self.assertEqual(len(road_path), 3)
        self.assertEqual(road_path[0], 8)
        self.assertEqual(road_path[1], 14)
        self.assertEqual(road_path[2], 15)

        road_path = rg.FindRoadPath(8, 2)
        self.assertEqual(len(road_path), 3)
        self.assertEqual(road_path[0], 8)
        self.assertEqual(road_path[1], 7)
        self.assertEqual(road_path[2], 2)

        road_path = rg.FindRoadPath(8, 3)
        self.assertEqual(len(road_path), 3)
        self.assertEqual(road_path[0], 8)
        self.assertEqual(road_path[1], 4)
        self.assertEqual(road_path[2], 3)

        road_path = rg.FindRoadPath(3, 2)
        self.assertEqual(len(road_path), 3)
        self.assertEqual(road_path[0], 3)
        self.assertEqual(road_path[1], 9)
        self.assertEqual(road_path[2], 2)

        road_path = rg.FindRoadPath(3, 15)
        self.assertEqual(len(road_path), 3)
        self.assertEqual(road_path[0], 3)
        self.assertEqual(road_path[1], 12)
        self.assertEqual(road_path[2], 15)

        road_path = rg.FindRoadPath(3, 8)
        self.assertEqual(len(road_path), 3)
        self.assertEqual(road_path[0], 3)
        self.assertEqual(road_path[1], 6)
        self.assertEqual(road_path[2], 8)

        road_path = rg.FindRoadPath(15, 2)
        self.assertEqual(len(road_path), 3)
        self.assertEqual(road_path[0], 15)
        self.assertEqual(road_path[1], 11)
        self.assertEqual(road_path[2], 2)

        road_path = rg.FindRoadPath(15, 8)
        self.assertEqual(len(road_path), 3)
        self.assertEqual(road_path[0], 15)
        self.assertEqual(road_path[1], 13)
        self.assertEqual(road_path[2], 8)

        road_path = rg.FindRoadPath(15, 3)
        self.assertEqual(len(road_path), 3)
        self.assertEqual(road_path[0], 15)
        self.assertEqual(road_path[1], 16)
        self.assertEqual(road_path[2], 3)

if __name__ == '__main__':
    unittest.main()
