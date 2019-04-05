# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import unittest
from bark.world.agent import *
from bark.models.behavior import *
from bark.world import *
from bark.world.map import *
from bark.models.dynamic import *
from bark.models.execution import *
from bark.geometry import *
from bark.geometry.standard_shapes import *
from bark.world.opendrive import *


class RoadgraphTests(unittest.TestCase):
    def test_world(self):
        road_graph = Roadgraph()
        lane1 = Lane()
        lane2 = Lane()
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

if __name__ == '__main__':
    unittest.main()
