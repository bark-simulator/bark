# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import unittest
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


class ImporterTests(unittest.TestCase):
    def test_python_map(self):
        pass
        # xodr_parser = XodrParser("modules/runtime/tests/data/Crossing8Course.xodr")
        # xodr_parser.print_python_map()

    def test_map(self):
        xodr_parser = XodrParser("modules/runtime/tests/data/urban_road.xodr")
        # xodr_parser = XodrParser("modules/runtime/tests/data/CulDeSac.xodr")
        params = ParameterServer()
        world = World(params)

        map_interface = MapInterface()
        map_interface.set_open_drive_map(xodr_parser.map)
        map_interface.set_roadgraph(xodr_parser.roadgraph)
        world.set_map(map_interface)


        for _, road in xodr_parser.map.get_roads().items():
            for lane_section in road.lane_sections:
                for _, lane in lane_section.get_lanes().items():

                    if lane.lane_type == LaneType.driving:
                        color="grey"
                    elif lane.lane_type == LaneType.sidewalk:
                        color="green"
                    else:
                        continue
                    
                    line_np = lane.line.toArray()
                    plt.text(line_np[-1, 0], line_np[-1, 1], 'center_{i}_{j}'.format(i=lane.lane_id,j=lane.lane_position))
                    
                    plt.plot(
                        line_np[:, 0],
                        line_np[:, 1],
                        color=color,
                        alpha=1.0)


        plt.axis("equal")
        plt.show()

        # driving corridor calculation test
        #lanes = map_interface.find_nearest_lanes(Point2d(-11,-8),1)
        #left_line, right_line, center_line = map_interface.calculate_driving_corridor(lanes[0].lane_id,2)
        #plt.plot(center_line.toArray()[:,0],center_line.toArray()[:,1])
        #plt.show()
        
        # TODO: plot cpp map
        #cwd = os.getcwd()
        #print (cwd)
        roadgraph = xodr_parser.roadgraph
        roadgraph.print_graph("/home/bernhard/"+"test1234.dot")
        


if __name__ == '__main__':
    unittest.main()
