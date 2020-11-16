# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import numpy as np
from bark.runtime.commons.parameters import ParameterServer
from bark.runtime.commons.xodr_parser import XodrParser
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
from bark.core.models.behavior import * 
import os


def make_initial_world(primitives):
    # must be within examples params folder
    params = ParameterServer()
    world = World(params)

    # Define two behavior models
    behavior_model = BehaviorMPContinuousActions(params)
    primitive_mapping = {}
    for prim in primitives:
      idx = behavior_model.AddMotionPrimitive(np.array(prim)) # adding action
      primitive_mapping[idx] = prim

    behavior_model.ActionToBehavior(0) # setting initial action

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

def get_ego_agent(world):
  return world.agents[0]

def apply_action_to_ego_agent(world, idx):
  get_ego_agent(world).behavior_model.ActionToBehavior(idx)
