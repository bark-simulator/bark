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
from bark.world.goal_definition import GoalDefinitionPolygon
from modules.runtime.viewer.matplotlib_viewer import MPViewer
from bark.world.evaluation import *


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
        newPlanView.add_line(Point2d(0, 0), 1.57079632679, 10)
        newXodrRoad.plan_view = newPlanView
        line = newXodrRoad.plan_view.get_reference_line().toArray()
        p = Point2d(line[-1][0], line[-1][1])
        newXodrRoad.plan_view.add_spiral(p, 1.57079632679, 50.0, 0.0, 0.3, 0.4)
        line = newXodrRoad.plan_view.get_reference_line()
        lane_section = XodrLaneSection(0)
        lane = XodrLane()
        lane.line = line
        lane_section.add_lane(lane)
        newXodrRoad.add_lane_section(lane_section)
        road_map.add_road(newXodrRoad)

        r = Roadgraph()
        map_interface = MapInterface()
        map_interface.set_open_drive_map(road_map)
        map_interface.set_roadgraph(r)
        world = World(params)
        world.add_agent(agent)

    
    def test_evaluator_drivable_area(self):


      # World Definition
      params = ParameterServer()
      world = World(params)

      # Model Definitions
      behavior_model = BehaviorConstantVelocity(params)
      execution_model = ExecutionModelInterpolate(params)
      dynamic_model = SingleTrackModel(params)

      # Map Definition
      map_interface = MapInterface()
      xodr_map = make_xodr_map_one_road_two_lanes()
      map_interface.set_open_drive_map(xodr_map)
      world.set_map(map_interface)
      #open_drive_map = world.map.get_open_drive_map()

      #agent_2d_shape = CarLimousine()
      agent_2d_shape = Polygon2d([1.25, 1, 0],[Point2d(-1,-1),Point2d(-1,1),Point2d(3,1), Point2d(3,-1)])
      init_state = np.array([0, 3, -1.75, 0, 5])
      agent_params = params.addChild("agent1")
      goal_polygon = Polygon2d([1, 1, 0],[Point2d(0,0),Point2d(0,2),Point2d(2,2), Point2d(2,0)])
      goal_polygon = goal_polygon.translate(Point2d(50, -2))

      agent = Agent(init_state,
                    behavior_model,
                    dynamic_model,
                    execution_model,
                    agent_2d_shape,
                    agent_params,
                    GoalDefinitionPolygon(goal_polygon), # goal_lane_id
                    map_interface)
      world.add_agent(agent)

      evaluator = EvaluatorDrivableArea()
      world.add_evaluator("drivable_area", evaluator)

      info = world.evaluate()
      self.assertFalse(info["drivable_area"])

      viewer = MPViewer(params=params,
                        use_world_bounds=True)

      # Draw map
      viewer.drawGoalDefinition(goal_polygon)
      viewer.drawWorld(world)
      viewer.drawRoadCorridor(agent.road_corridor)
      viewer.show(block=False)

if __name__ == '__main__':
    unittest.main()
