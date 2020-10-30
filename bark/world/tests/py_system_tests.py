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
from bark.core.models.execution import ExecutionModelInterpolate
from bark.core.models.dynamic import SingleTrackModel, StateDefinition
from bark.core.world import World, MakeTestWorldHighway
from bark.core.world.goal_definition import GoalDefinitionPolygon, GoalDefinitionStateLimitsFrenet
from bark.core.world.agent import Agent
from bark.core.world.map import MapInterface, Roadgraph
from bark.core.geometry.standard_shapes import CarLimousine, CarRectangle
from bark.core.geometry import Point2d, Polygon2d, Line2d
from bark.core.world.evaluation import EvaluatorDrivableArea
from bark.core.world.opendrive import OpenDriveMap, XodrRoad, PlanView, \
    MakeXodrMapOneRoadTwoLanes, XodrLaneSection, XodrLane
from bark.runtime.viewer.video_renderer import VideoRenderer
from bark.core.models.behavior import * 
import os

from bark.world.tests.python_behavior_model import PythonDistanceBehavior

class SystemTests(unittest.TestCase):
    """ This shall serve as a full system test, importing world, agent, and behavior models
    """
    #@unittest.skip
    def test_uct_single_agent(self):
        try:
            from bark.core.models.behavior import BehaviorUCTSingleAgentMacroActions
        except:
            print("Rerun with --define planner_uct=true")
            return
        # World Definition
        scenario_param_file ="macro_actions_test.json" # must be within examples params folder
        params = ParameterServer(filename= os.path.join(os.path.dirname(__file__),"params/",scenario_param_file))

        world = World(params)

        # Model Definitions
        behavior_model = BehaviorUCTSingleAgentMacroActions(params)
        execution_model = ExecutionModelInterpolate(params)
        dynamic_model = SingleTrackModel(params)

        behavior_model2 = BehaviorConstantAcceleration(params)
        execution_model2 = ExecutionModelInterpolate(params)
        dynamic_model2 = SingleTrackModel(params)

        # Map Definition
        map_interface = MapInterface()
        xodr_map = MakeXodrMapOneRoadTwoLanes()
        map_interface.SetOpenDriveMap(xodr_map)
        world.SetMap(map_interface)

        # agent_2d_shape = CarLimousine()
        agent_2d_shape = CarRectangle()
        init_state = np.array([0, 3, -5.25, 0, 20])
        agent_params = params.AddChild("agent1")


        # goal_polygon = Polygon2d(
        #     [1, 1, 0], [Point2d(0, 0), Point2d(0, 2), Point2d(2, 2), Point2d(2, 0)])
        # goal_definition = GoalDefinitionPolygon(goal_polygon)
        # goal_polygon = goal_polygon.Translate(Point2d(90, -2))

        center_line = Line2d()
        center_line.AddPoint(Point2d(0.0, -1.75))
        center_line.AddPoint(Point2d(100.0, -1.75))

        max_lateral_dist = (0.4,0.5)
        max_orientation_diff = (0.08, 0.1)
        velocity_range = (5.0, 20.0)
        goal_definition = GoalDefinitionStateLimitsFrenet(center_line,
                        max_lateral_dist, max_orientation_diff,
                        velocity_range)

        agent = Agent(init_state, behavior_model, dynamic_model, execution_model,
                      agent_2d_shape, agent_params, goal_definition, map_interface)
        world.AddAgent(agent)

        init_state2 = np.array([0, 25, -5.25, 0, 0])
        agent2 = Agent(init_state2, behavior_model2, dynamic_model2, execution_model2,
                        agent_2d_shape, agent_params, goal_definition, map_interface)
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

        for _ in range(0, 5):
            world.Step(sim_step_time)
            viewer.clear()
            video_renderer.drawWorld(world)
            video_renderer.drawGoalDefinition(goal_definition)
            time.sleep(sim_step_time/sim_real_time_factor)

        video_renderer.export_video(filename="./test_video_intermediate", remove_image_dir=True)


    def test_python_behavior_model(self):
        # World Definition
        scenario_param_file ="macro_actions_test.json" # must be within examples params folder
        params = ParameterServer(filename= os.path.join(os.path.dirname(__file__),"params/",scenario_param_file))

        world = World(params)

        # Define two behavior models one python one standard c++ model
        behavior_model = PythonDistanceBehavior(params)
        execution_model = ExecutionModelInterpolate(params)
        dynamic_model = SingleTrackModel(params)

        behavior_model2 = BehaviorConstantAcceleration(params)
        execution_model2 = ExecutionModelInterpolate(params)
        dynamic_model2 = SingleTrackModel(params)
        
        # Define the map interface and load a testing map
        map_interface = MapInterface()
        xodr_map = MakeXodrMapOneRoadTwoLanes()
        map_interface.SetOpenDriveMap(xodr_map)
        world.SetMap(map_interface)

        # Define the agent shapes
        agent_2d_shape = CarRectangle()
        init_state = np.array([0, 3, -5.25, 0, 20])
        
        # Define the goal definition for agents
        center_line = Line2d()
        center_line.AddPoint(Point2d(0.0, -1.75))
        center_line.AddPoint(Point2d(100.0, -1.75))

        max_lateral_dist = (0.4,0.5)
        max_orientation_diff = (0.08, 0.1)
        velocity_range = (5.0, 20.0)
        goal_definition = GoalDefinitionStateLimitsFrenet(center_line,
                        max_lateral_dist, max_orientation_diff,
                        velocity_range)

        # define two agents with the different behavior models
        agent_params = params.AddChild("agent1")
        agent = Agent(init_state, behavior_model, dynamic_model, execution_model,
                      agent_2d_shape, agent_params, goal_definition, map_interface)
        world.AddAgent(agent)

        init_state2 = np.array([0, 25, -5.25, 0, 15])
        agent2 = Agent(init_state2, behavior_model2, dynamic_model2, execution_model2,
                        agent_2d_shape, agent_params, goal_definition, map_interface)
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
            viewer.clear()
            video_renderer.drawWorld(world)
            video_renderer.drawGoalDefinition(goal_definition, "red", 0.5, "red")
            time.sleep(sim_step_time/sim_real_time_factor)

        video_renderer.export_video(filename="./test_video_intermediate", remove_image_dir=True)

if __name__ == '__main__':
    unittest.main()
