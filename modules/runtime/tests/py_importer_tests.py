# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import unittest
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

def helper_plot(xodr_parser):
    for _, road in xodr_parser.map.get_roads().items():
        for lane_section in road.lane_sections:
            for _, lane in lane_section.get_lanes().items():

                if lane.lane_type == LaneType.driving:
                    color="grey"
                elif lane.lane_type == LaneType.sidewalk:
                    color="green"
                elif lane.lane_type == LaneType.border:
                    color="red"
                elif lane.lane_type == LaneType.none:
                    color="blue"
                else:
                    continue
                
                line_np = lane.line.toArray()
                
                #print(lane.road_mark)
                plt.text(line_np[-1, 0], line_np[-1, 1], 'center_{i}_{j}'.format(i=lane.lane_id,j=lane.lane_position))
                
                plt.plot(
                    line_np[:, 0],
                    line_np[:, 1],
                    color=color,
                    alpha=1.0)


    plt.axis("equal")
    plt.show()


class ImporterTests(unittest.TestCase):
    def test_python_map(self):
        pass
        # xodr_parser = XodrParser("modules/runtime/tests/data/Crossing8Course.xodr")
        # xodr_parser.print_python_map()

    def test_map_CulDeSac(self):
        xodr_parser = XodrParser("modules/runtime/tests/data/CulDeSac.xodr")
        #dot_file_path = "/home/esterle/roadgraph/" + "CulDeSac_temp.dot"
        dot_file_path = "CulDeSac.dot"

        params = ParameterServer()
        world = World(params)

        map_interface = MapInterface()
        map_interface.set_open_drive_map(xodr_parser.map)
        world.set_map(map_interface)
        
        #helper_plot(xodr_parser)

        roadgraph = map_interface.get_roadgraph()
        
        roadgraph.print_graph(dot_file_path)
        self.assertTrue(filecmp.cmp("modules/runtime/tests/data/CulDeSac_ideal.dot", dot_file_path, shallow=False))


if __name__ == '__main__':
    unittest.main()
