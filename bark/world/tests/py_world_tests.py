# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import unittest
import numpy as np
import time
from bark.runtime.commons import ParameterServer
from bark.runtime.viewer import MPViewer
from bark.runtime.commons import XodrParser
from bark.core.models.behavior import BehaviorConstantAcceleration, BehaviorIDMLaneTracking, BehaviorMobilRuleBased
from bark.core.models.execution import ExecutionModelInterpolate
from bark.core.models.dynamic import SingleTrackModel
from bark.core.world import World, MakeTestWorldHighway
from bark.core.world.goal_definition import GoalDefinitionPolygon
from bark.core.world.agent import Agent
from bark.core.world.map import MapInterface, Roadgraph
from bark.core.geometry.standard_shapes import CarLimousine, CarRectangle
from bark.core.geometry import Point2d, Polygon2d
from bark.core.world.evaluation import EvaluatorDrivableArea
from bark.core.world.opendrive import OpenDriveMap, XodrRoad, PlanView, \
    MakeXodrMapOneRoadTwoLanes, XodrLaneSection, XodrLane


class WorldTests(unittest.TestCase):
    """This test should create and obstacle, map and agent insert it into the world.
      FM, the world should be stepped several times in order to verify the functionality.
    """

    def test_world(self):
        # create agent
        params = ParameterServer()
        behavior = BehaviorConstantAcceleration(params)
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
        newPlanView.AddLine(Point2d(0, 0), 1.57079632679, 10, 10)
        newXodrRoad.plan_view = newPlanView
        line = newXodrRoad.plan_view.GetReferenceLine().ToArray()
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

    def test_highway(self):
        world = MakeTestWorldHighway()

    def test_evaluator_drivable_area(self):
        # World Definition
        params = ParameterServer()
        world = World(params)

        # Model Definitions
        behavior_model = BehaviorConstantAcceleration(params)
        execution_model = ExecutionModelInterpolate(params)
        dynamic_model = SingleTrackModel(params)

        # Map Definition
        map_interface = MapInterface()
        xodr_map = MakeXodrMapOneRoadTwoLanes()
        map_interface.SetOpenDriveMap(xodr_map)
        world.SetMap(map_interface)
        #open_drive_map = world.map.GetOpenDriveMap()

        #agent_2d_shape = CarLimousine()
        agent_2d_shape = Polygon2d(
            [1.25, 1, 0], [Point2d(-1, -1), Point2d(-1, 1), Point2d(3, 1), Point2d(3, -1)])
        init_state = np.array([0, 3, -1.75, 0, 5])
        agent_params = params.AddChild("agent1")
        goal_polygon = Polygon2d(
            [1, 1, 0], [Point2d(0, 0), Point2d(0, 2), Point2d(2, 2), Point2d(2, 0)])
        goal_polygon = goal_polygon.Translate(Point2d(50, -2))

        agent = Agent(init_state,
                      behavior_model,
                      dynamic_model,
                      execution_model,
                      agent_2d_shape,
                      agent_params,
                      GoalDefinitionPolygon(goal_polygon),  # goal_lane_id
                      map_interface)
        world.AddAgent(agent)

        evaluator = EvaluatorDrivableArea()
        world.AddEvaluator("drivable_area", evaluator)

        info = world.Evaluate()
        self.assertFalse(info["drivable_area"])

        viewer = MPViewer(params=params,
                          use_world_bounds=True)

        # Draw map
        viewer.drawGoalDefinition(goal_polygon, color=(1,0,0), alpha=0.5, facecolor= (1,0,0))
        viewer.drawWorld(world)
        viewer.drawRoadCorridor(agent.road_corridor)
        viewer.show(block=False)

    def test_lane_change(self):
        # World Definition
        params = ParameterServer()
        world = World(params)

        # Model Definitions
        behavior_model = BehaviorMobilRuleBased(params)
        execution_model = ExecutionModelInterpolate(params)
        dynamic_model = SingleTrackModel(params)

        behavior_model2 = BehaviorIDMLaneTracking(params)
        execution_model2 = ExecutionModelInterpolate(params)
        dynamic_model2 = SingleTrackModel(params)

        # Map Definition
        map_interface = MapInterface()
        xodr_map = MakeXodrMapOneRoadTwoLanes()
        map_interface.SetOpenDriveMap(xodr_map)
        world.SetMap(map_interface)

        #agent_2d_shape = CarLimousine()
        agent_2d_shape = CarRectangle()
        init_state = np.array([0, 3, -1.75, 0, 5])
        agent_params = params.AddChild("agent1")
        goal_polygon = Polygon2d(
            [1, 1, 0], [Point2d(0, 0), Point2d(0, 2), Point2d(2, 2), Point2d(2, 0)])
        goal_polygon = goal_polygon.Translate(Point2d(50, -2))

        agent = Agent(init_state, behavior_model, dynamic_model, execution_model,
                      agent_2d_shape, agent_params, GoalDefinitionPolygon(goal_polygon), map_interface)
        world.AddAgent(agent)

        init_state2 = np.array([0, 15, -1.75, 0, 2])
        agent2 = Agent(init_state2, behavior_model2, dynamic_model2, execution_model2,
                       agent_2d_shape, agent_params, GoalDefinitionPolygon(goal_polygon), map_interface)
        world.AddAgent(agent2)

        # viewer
        viewer = MPViewer(params=params, use_world_bounds=True)

        # World Simulation
        sim_step_time = params["simulation"]["step_time",
                                             "Step-time in simulation", 0.05]
        sim_real_time_factor = params["simulation"]["real_time_factor",
                                                    "execution in real-time or faster", 100]

        # Draw map
        for _ in range(0, 10):
            viewer.clear()
            world.Step(sim_step_time)
            viewer.drawWorld(world)
            viewer.show(block=False)
            time.sleep(sim_step_time/sim_real_time_factor)


if __name__ == '__main__':
    unittest.main()
