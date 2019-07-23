import unittest
import numpy as np

from bark.world.map import MapInterface
from bark.geometry import distance, Point2d

from modules.runtime.commons.xodr_parser import XodrParser


def get_min_distance_between_lines(line1, line2):
    """
    Calculate the distances between all combinations of endpoints of the two lines, and return the smallest.
    This is necessary because one or both lines could be reversed.
    """

    first_point1 = Point2d(line1[0, 0], line1[0, 1])
    last_point1 = Point2d(line1[-1, 0], line1[-1, 1])
    first_point2 = Point2d(line2[0, 0], line2[0, 1])
    last_point2 = Point2d(line2[-1, 0], line2[-1, 1])

    distance_first_first = distance(first_point1, first_point2)
    distance_first_last = distance(first_point1, last_point2)
    distance_last_first = distance(last_point1, first_point2)
    distance_last_last = distance(last_point1, last_point2)

    return min(distance_first_first, distance_first_last, distance_last_first, distance_last_last)


class DisconnectedLinesTest(unittest.TestCase):
    def test_disconnected_lines_for_4way_intersection(self):
        xodr_file = "modules/runtime/tests/data/4way_intersection.xodr"
        xodr_parser = XodrParser(xodr_file)

        roadgraph = xodr_parser.roadgraph
        opendrive_map = xodr_parser.map

        for vertex_t in roadgraph.get_vertices():
            vertex = roadgraph.get_vertex(vertex_t)
            lane = vertex.lane
            line = lane.line.toArray()
            for successor_id in roadgraph.get_successor_lanes(lane.lane_id):
                successor_vertex_t, _ = roadgraph.get_vertex_by_lane_id(successor_id)
                successor_vertex = roadgraph.get_vertex(successor_vertex_t)
                successor_lane = successor_vertex.lane
                successor = successor_lane.line.toArray()

                min_distance = get_min_distance_between_lines(line, successor)
                self.assertAlmostEqual(
                    min_distance,
                    0,
                    msg="The distance between the last point of lane {} and the first point of lane {} is != 0".format(
                        lane.lane_id, successor_lane.lane_id)
                )
                

if __name__ == "__main__":
    unittest.main()
