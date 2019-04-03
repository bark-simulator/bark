# Copyright (c) 2019 fortiss GmbH
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import unittest
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib as mpl
import pickle
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
from modules.runtime.runtime import Runtime
from modules.runtime.commons.xodr_parser import XodrParser
from modules.runtime.commons.roadgraph_generator import RoadgraphGenerator


class ImporterTests(unittest.TestCase):
    def test_python_map(self):
        pass
        #xodr_parser = XodrParser("modules/runtime/tests/data/Crossing8Course.xodr")
        #xodr_parser.print_python_map()

    def test_map(self):
        xodr_parser = XodrParser("modules/runtime/tests/data/Crossing8Course.xodr")
        #xodr_parser = XodrParser("modules/runtime/tests/data/CulDeSac.xodr")
        params = ParameterServer()
        world = World(params)

        map_interface = MapInterface()
        map_interface.set_open_drive_map(xodr_parser.map)
        map_interface.set_roadgraph(xodr_parser.roadgraph)
        world.set_map(map_interface)


        for i, road in xodr_parser.map.get_roads().items():
            """
            pv_np = road.plan_view.get_reference_line().toArray()
            m = cm.ScalarMappable(
                norm=mpl.colors.Normalize(vmin=501, vmax=517), cmap=cm.jet)
            plt.plot(pv_np[:, 0], pv_np[:, 1])
            plt.text(pv_np[-1, 0], pv_np[-1, 1], 'sample {i}'.format(i=i))
            """
            for lane_section in road.lane_sections:
                for _, lane in lane_section.get_lanes().items():
                    line_np = lane.line.toArray()
                    plt.text(line_np[-1, 0], line_np[-1, 1], 'center_{i}_{j}'.format(i=lane.lane_id,j=lane.lane_position))
                    plt.plot(
                        line_np[:, 0],
                        line_np[:, 1],
                        color="grey",
                        alpha=1.0)


        plt.axis("equal")
        plt.show()

        
        # TODO: plot cpp map
        #cwd = os.getcwd()
        #print (cwd)
        #roadgraph.print_graph("/home/kessler/"+"test1234.dot")
        

    
    """
    def test_map(self):
        xodr_parser = XodrParser("tests/runtime/data/Crossing8Course.xodr")
        for i, road in xodr_parser.map.get_roads().items():
            pv_np = road.plan_view.get_reference_line().toArray()
            m = cm.ScalarMappable(
                norm=mpl.colors.Normalize(vmin=501, vmax=517), cmap=cm.jet)
            plt.plot(pv_np[:, 0], pv_np[:, 1], color=m.to_rgba(i))
            plt.text(pv_np[-1, 0], pv_np[-1, 1], 'sample {i}'.format(i=i))

            for lane_section in road.get_lane_sections:
                for _, lane in lane_section.get_lanes().items():
                    line_center_np = lane.center_line.toArray()
                    plt.text(line_center_np[-1, 0], line_center_np[-1, 1], 'center_{i}_{j}'.format(i=lane.lane_id,j=lane.lane_position))
                    line_left_np = lane.left_line.toArray()
                    plt.text(line_left_np[-1, 0], line_left_np[-1, 1], 'left_{i}_{j}'.format(i=lane.lane_id,j=lane.lane_position))
                    line_right_np = lane.right_line.toArray()
                    plt.text(line_right_np[-1, 0], line_right_np[-1, 1], 'right_ {i}_{j}'.format(i=lane.lane_id,j=lane.lane_position))
                    plt.plot(
                        line_center_np[:, 0],
                        line_center_np[:, 1],
                        color="red",
                        alpha=1.0)
                    plt.plot(
                        line_left_np[:, 0],
                        line_left_np[:, 1],
                        color="blue",
                        alpha=1.0)
                    plt.plot(
                        line_right_np[:, 0],
                        line_right_np[:, 1],
                        color="green",
                        alpha=1.0)

        ax = plt.gca()

        # with open("tests/runtime/data/importer_test.data", "wb") as f:  #Creates test-data
          # pickle.dump(ax.lines, f)
        plt.show()
        
        with open("tests/runtime/data/importer_test.data", "rb") as f:
            saved_lines = pickle.load(f)
        self.assertTrue(any(saved_lines))

        found_lines = []
        for line_test in ax.lines:
            test_xy = line_test.get_xydata()
            found_line = []
            for line_true in saved_lines:
                true_xy = line_true.get_xydata()
                if (true_xy.shape[0] == test_xy.shape[0]
                        and true_xy.shape[1] == test_xy.shape[1]):
                    if (np.allclose(line_true.get_xydata(),
                                    line_test.get_xydata())):
                        found_line.append(True)

            found_lines.append(
                any(found_line))  # Checks if a single matching line is found
        found_lines.append(any(found_lines))  # Checks if empty
        self.assertTrue(all(found_lines))
    """

if __name__ == '__main__':
    unittest.main()
