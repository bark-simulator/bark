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
        newXodrRoad = XodrRoad()
        newXodrRoad.id = 1
        newXodrRoad.name = "Autobahn A9"
        newPlanView = PlanView()
        newPlanView.AddLine(Point2d(0, 0), 1.57079632679, 10)
        newXodrRoad.plan_view = newPlanView
        line = newXodrRoad.plan_view.GetReferenceLine().toArray()
        p = Point2d(line[-1][0], line[-1][1])
        newXodrRoad.plan_view.AddSpiral(p, 1.57079632679, 50.0, 0.0, 0.3, 0.4)
        line = newXodrRoad.plan_view.GetReferenceLine()
        lane_section = XodrLaneSection(0)
        lane = XodrLane()
        lane.line = line
        lane_section.AddLane(lane)
        newXodrRoad.AddLaneSection(lane_section)
        road_map.AddRoad(newXodrRoad)

        r = Roadgraph()
        map_interface = MapInterface()
        map_interface.SetOpenDriveMap(road_map)
        map_interface.SetRoadgraph(r)
        world = World(params)
        world.AddAgent(agent)


if __name__ == '__main__':
    unittest.main()
