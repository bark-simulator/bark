# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT


import numpy as np
import time
import os
from bark.world.agent import *
from bark.models.behavior import *
from bark.world import *
from bark.world.map import *
from bark.world.goal_definition import GoalDefinitionPolygon
from bark.models.dynamic import *
from bark.models.execution import *
from bark.geometry import *
from bark.geometry.standard_shapes import *
from modules.runtime.commons.parameters import ParameterServer
from modules.runtime.viewer.pygame_viewer import PygameViewer
from modules.runtime.viewer.matplotlib_viewer import MPViewer
from modules.runtime.viewer.panda3d_viewer import Panda3dViewer
from modules.runtime.commons.xodr_parser import XodrParser


# Parameters Definitions
param_server = ParameterServer(filename="examples/params/od8_const_vel_one_agent.json")
# set parameter that is accessible in Python as well as cpp
# param_server.setReal("wheel_base", 0.8)

# World Definition
world = World(param_server)

# Model Definitions
behavior_model = BehaviorConstantVelocity(param_server)
execution_model = ExecutionModelInterpolate(param_server)
dynamic_model = SingleTrackModel(param_server)

# Map Definition
xodr_parser = XodrParser("modules/runtime/tests/data/Crossing8Course.xodr")
map_interface = MapInterface()
map_interface.set_open_drive_map(xodr_parser.map)
world.set_map(map_interface)
# Agent Definition
agent_2d_shape = CarLimousine()
init_state = np.array([0, -11, -8, 3.14*3.0/4.0, 10/3.6])
agent_params = param_server.addChild("agent1")
goal_polygon = Polygon2d([0, 0, 0],[Point2d(-1,-1),Point2d(-1,1),Point2d(1,1), Point2d(1,-1)])
goal_polygon = goal_polygon.translate(Point2d(-191.789,-50.1725))

agent = Agent(init_state,
              behavior_model,
              dynamic_model,
              execution_model,
              agent_2d_shape,
              agent_params,
              GoalDefinitionPolygon(goal_polygon), # goal_lane_id
              map_interface)
world.add_agent(agent)

# viewer
viewer = PygameViewer(params=param_server,
                      x_range=[-50, 50],
                      y_range=[-50, 50],
                      follow_agent_id=agent.id,
                      screen_dims=[500, 500])

"""
viewer = Panda3dViewer(params=param_server,
                      x_range=[-150, 150],
                      y_range=[-150, 150],
                      follow_agent_id=agent.id)
"""
viewer = MPViewer(params=param_server)

# World Simulation
sim_step_time = param_server["simulation"]["step_time",
                                           "Step-time in simulation",
                                           0.05]
sim_real_time_factor = param_server["simulation"]["real_time_factor",
                                                  "execution in real-time or faster",
                                                  100]

for _ in range(0, 100):
    viewer.clear()
    world.step(sim_step_time)
    viewer.drawWorld(world)
    viewer.show(block=False)
    time.sleep(sim_step_time/sim_real_time_factor)

param_server.save("examples/params/od8_const_vel_one_agent_written.json")