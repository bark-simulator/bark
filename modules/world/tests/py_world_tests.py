# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import unittest
import numpy as np
from bark.world.agent import *
from bark.models.behavior import *
from bark.world import *
from bark.world.map import *
from bark.models.dynamic import *
from bark.models.execution import *
from bark.geometry import *
from bark.geometry.standard_shapes import *
from modules.runtime.commons.parameters import ParameterServer
from bark.world.opendrive import *


class WorldTests(unittest.TestCase):
    """This test should create and obstacle, map and agent insert it into the world.
     FM, the world should be stepped several times in order to verify the functionality.
    """

    def test_world(self):

        # create agent
        params = ParameterServer()
        behavior = BehaviorConstantVelocity(params)
        execution = ExecutionModelInterpolate(params)
        dynamic = SingleTrackModel(params)
        shape = Polygon2d([1.25, 1, 0], [
            Point2d(0, 0),
            Point2d(0, 2),
            Point2d(4, 2),
            Point2d(4, 0),
            Point2d(0, 0)
        ])
        init_state = np.array([0, 0, 0, 0, 5])
        agent = Agent(init_state, behavior, dynamic, execution, shape,
                      params.AddChild("agent"))
        road_map = OpenDriveMap()
        newRoad = Road()
        newRoad.id = 1
        newRoad.name = "Autobahn A9"
        newPlanView = PlanView()
        newPlanView.add_line(Point2d(0, 0), 1.57079632679, 10)
        newRoad.plan_view = newPlanView
        line = newRoad.plan_view.get_reference_line().toArray()
        p = Point2d(line[-1][0], line[-1][1])
        newRoad.plan_view.add_spiral(p, 1.57079632679, 50.0, 0.0, 0.3, 0.4)
        line = newRoad.plan_view.get_reference_line()
        lane_section = LaneSection(0)
        lane = Lane()
        lane.line = line
        lane_section.add_lane(lane)
        newRoad.add_lane_section(lane_section)
        road_map.add_road(newRoad)

        r = Roadgraph()
        map_interface = MapInterface()
        map_interface.set_open_drive_map(road_map)
        map_interface.set_roadgraph(r)
        world = World(params)
        world.add_agent(agent)


if __name__ == '__main__':
    unittest.main()
