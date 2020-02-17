# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import unittest
import numpy as np
import time
from modules.runtime.commons.parameters import ParameterServer
from modules.runtime.viewer.matplotlib_viewer import MPViewer
from modules.runtime.commons.xodr_parser import XodrParser
try:
  from bark.models.behavior import BehaviorConstantVelocity, BehaviorMobil, BehaviorUCTSingleAgent
except:
  print("Rerun with --define planner_uct=true")
  exit()
from bark.models.execution import ExecutionModelInterpolate
from bark.models.dynamic import SingleTrackModel
from bark.world import World, MakeTestWorldHighway
from bark.world.goal_definition import GoalDefinitionPolygon
from bark.world.agent import Agent
from bark.world.map import MapInterface, Roadgraph
from bark.geometry.standard_shapes import CarLimousine, CarRectangle
from bark.geometry import Point2d, Polygon2d
from bark.world.evaluation import EvaluatorDrivableArea
from bark.world.opendrive import OpenDriveMap, XodrRoad, PlanView, \
    MakeXodrMapOneRoadTwoLanes, XodrLaneSection, XodrLane
from modules.runtime.viewer.video_renderer import VideoRenderer

import os


class SystemTests(unittest.TestCase):
    """ This shall serve as a full system test, importing world, agent, and behavior models
    """
    #@unittest.skip
    def test_uct_single_agent(self):
        # World Definition
        scenario_param_file ="uct_planner.json" # must be within examples params folder
        params = ParameterServer(filename= os.path.join("examples/params/",scenario_param_file))

        world = World(params)

        # Model Definitions
        behavior_model = BehaviorUCTSingleAgent(params)
        execution_model = ExecutionModelInterpolate(params)
        dynamic_model = SingleTrackModel(params)

        behavior_model2 = BehaviorConstantVelocity(params)
        execution_model2 = ExecutionModelInterpolate(params)
        dynamic_model2 = SingleTrackModel(params)

        # Map Definition
        map_interface = MapInterface()
        xodr_map = MakeXodrMapOneRoadTwoLanes()
        map_interface.SetOpenDriveMap(xodr_map)
        world.SetMap(map_interface)

        # agent_2d_shape = CarLimousine()
        agent_2d_shape = CarRectangle()
        init_state = np.array([0, 3, -1.75, 0, 5])
        agent_params = params.addChild("agent1")
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
                                              "Step-time in simulation", 0.2]
        sim_real_time_factor = params["simulation"]["real_time_factor",
                                                    "execution in real-time or faster", 1]

        # Draw map
        video_renderer = VideoRenderer(renderer=viewer, world_step_time=sim_step_time)

        for _ in range(0, 20):
            world.Step(sim_step_time)
            video_renderer.drawWorld(world)
            video_renderer.drawGoalDefinition(GoalDefinitionPolygon(goal_polygon))
            time.sleep(sim_step_time/sim_real_time_factor)

        video_renderer.export_video(filename="./test_video_intermediate", remove_image_dir=True)


if __name__ == '__main__':
    unittest.main()
