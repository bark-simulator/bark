# Copyright (c) 2020 Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import numpy as np
from modules.runtime.commons.parameters import ParameterServer
from modules.runtime.commons.xodr_parser import XodrParser
from bark.models.execution import ExecutionModelInterpolate
from bark.models.dynamic import SingleTrackModel, StateDefinition
from bark.world import World, MakeTestWorldHighway
from bark.world.goal_definition import GoalDefinitionPolygon, GoalDefinitionStateLimitsFrenet
from bark.world.agent import Agent
from bark.world.map import MapInterface, Roadgraph
from bark.geometry.standard_shapes import CarLimousine, CarRectangle
from bark.geometry import Point2d, Polygon2d, Line2d
from bark.world.evaluation import EvaluatorDrivableArea
from bark.world.opendrive import OpenDriveMap, XodrRoad, PlanView, \
    MakeXodrMapOneRoadTwoLanes, XodrLaneSection, XodrLane
from bark.models.behavior import * 
import os

def make_initial_world():
    # must be within examples params folder
    params = ParameterServer()
    world = World(params)

    # Define two behavior models one python one standard c++ model
    behavior_model = BehaviorMPContinuousActions(params)
    idx = behavior_model.AddMotionPrimitive(np.array([-5, 0])) # adding action
    behavior_model.ActionToBehavior(idx) # setting action

    execution_model = ExecutionModelInterpolate(params)
    dynamic_model = SingleTrackModel(params)

    behavior_model2 = BehaviorConstantVelocity(params)
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

    max_lateral_dist = (0.4, 0.5)
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

    return world

def get_controlled_agent(world):
  return world.agents[0]
