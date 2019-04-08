# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import unittest
import numpy as np
from bark.world.agent import *
from bark.models.behavior import *
from bark.world import *
from bark.models.dynamic import *
from bark.models.execution import *
from bark.geometry import *
from bark.geometry.standard_shapes import *
from modules.runtime.commons.parameters import ParameterServer
from modules.runtime.commons.roadgraph_generator import RoadgraphGenerator
from bark.world.opendrive import *
from bark.world.map import *
from modules.runtime.runtime import Runtime


class RuntimeTests(unittest.TestCase):

    def test_runtime(self):
        # create agent

        # initialize models
        params = ParameterServer()
        behavior = BehaviorConstantVelocity(params)
        execution = ExecutionModelMpc(params)
        dynamic = SingleTrackModel()
        shape = Polygon2d([1.25, 1, 0], [
            Point2d(0, 0),
            Point2d(0, 2),
            Point2d(4, 2),
            Point2d(4, 0),
            Point2d(0, 0)
        ])
        init_state = np.array([0, 0, 0, 0, 5])
        agent = Agent(init_state, behavior, dynamic, execution, shape,
                      params.AddChild("agent"), lane_id = 0)

        # create map (1x Road, 1x LaneSection, 1x Lane)
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

        # set roadgraph
        roadgraph = Roadgraph()
        rg = RoadgraphGenerator(roadgraph)
        rg.generate(road_map)

        # create world
        world = World(params)
        world.add_agent(agent)
        map_interface = MapInterface()
        map_interface.set_open_drive_map(road_map)
        map_interface.set_roadgraph(roadgraph)
        world.set_map(map_interface)


        # step the world
        # TEST obsolete; see examples/
        """
        runtime = Runtime(world, 0.25)
        runtime.run(20)

        for _, agent in runtime.world.agents.items():
            print(agent.followed_trajectory)
        """



if __name__ == '__main__':
    unittest.main()
