# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import numpy as np
import time
import os
from bark.runtime.commons import ParameterServer
from bark.runtime.viewer import MPViewer
from bark.runtime.commons import XodrParser
from bark.pybark.core.models.behavior import BehaviorConstantVelocity
from bark.pybark.core.models.execution import ExecutionModelInterpolate
from bark.pybark.core.models.dynamic import SingleTrackModel
from bark.pybark.core.world import World
from bark.pybark.core.world.goal_definition import GoalDefinitionPolygon
from bark.pybark.core.world.agent import Agent
from bark.pybark.core.world.map import MapInterface
from bark.pybark.core.geometry.standard_shapes import CarLimousine
from bark.pybark.core.geometry import Point2d, Polygon2d


# Parameters Definitions
param_server = ParameterServer()

# World Definition
world = World(param_server)

# Model Definitions
behavior_model = BehaviorConstantVelocity(param_server)
execution_model = ExecutionModelInterpolate(param_server)
dynamic_model = SingleTrackModel(param_server)

behavior_model2 = BehaviorConstantVelocity(param_server)
execution_model2 = ExecutionModelInterpolate(param_server)
dynamic_model2 = SingleTrackModel(param_server)

# Map Definition
xodr_parser = XodrParser(os.path.join(os.path.dirname(__file__), "../runtime/tests/data/Crossing8Course.xodr"))
map_interface = MapInterface()
map_interface.SetOpenDriveMap(xodr_parser.map)
world.SetMap(map_interface)

# Agent Definition
agent_2d_shape = CarLimousine()
init_state = np.array([0, -15, -13, 3.14*3.0/4.0, 50/3.6])
goal_polygon = Polygon2d([0, 0, 0],[Point2d(-1,-1),Point2d(-1,1),Point2d(1,1), Point2d(1,-1)])
goal_polygon = goal_polygon.Translate(Point2d(-63,-61))
agent_params = param_server.addChild("agent1")
agent1 = Agent(init_state,
               behavior_model,
               dynamic_model,
               execution_model,
               agent_2d_shape,
               agent_params,
               GoalDefinitionPolygon(goal_polygon),
               map_interface)
world.AddAgent(agent1)

agent_2d_shape2 = CarLimousine()
init_state2 = np.array([0, -15, -13, 3.14*3.0/4.0, 5.2])
agent_params2 = param_server.addChild("agent2")
agent2 = Agent(init_state2,
               behavior_model2,
               dynamic_model2,
               execution_model2,
               agent_2d_shape2,
               agent_params2,
               GoalDefinitionPolygon(goal_polygon),
               map_interface)
world.AddAgent(agent2)

# viewer
viewer = MPViewer(params=param_server, use_world_bounds=True)

# World Simulation
sim_step_time = param_server["simulation"]["step_time",
                                           "Step-time used in simulation",
                                           1]
sim_real_time_factor = param_server["simulation"]["real_time_factor",
                                                  "execution in real-time or faster",
                                                  1]

for _ in range(0, 10):
  world.Step(sim_step_time)
  viewer.drawWorld(world)
  viewer.show(block=False)
  time.sleep(sim_step_time/sim_real_time_factor)

param_server.save(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                  "params",
                  "od8_const_vel_one_agent_written.json"))