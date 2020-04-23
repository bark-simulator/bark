# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import numpy as np
import time
import os
from modules.runtime.commons.parameters import ParameterServer
from modules.runtime.viewer.matplotlib_viewer import MPViewer
from modules.runtime.commons.xodr_parser import XodrParser
from bark.models.behavior import BehaviorConstantVelocity, BehaviorIDMClassic
from bark.models.execution import ExecutionModelInterpolate
from bark.models.dynamic import SingleTrackModel
from bark.world import World
from bark.world.goal_definition import GoalDefinitionPolygon
from bark.world.agent import Agent
from bark.world.map import MapInterface
from bark.geometry.standard_shapes import CarLimousine
from bark.geometry import Point2d, Polygon2d


# Parameters Definitions
param_server = ParameterServer()

# World Definition
world = World(param_server)

# Model Definitions
# behavior_model = BehaviorConstantVelocity(param_server)
behavior_model = BehaviorIDMClassic(param_server)
execution_model = ExecutionModelInterpolate(param_server)
dynamic_model = SingleTrackModel(param_server)

# behavior_model2 = BehaviorConstantVelocity(param_server)
behavior_model2 = BehaviorIDMClassic(param_server)
execution_model2 = ExecutionModelInterpolate(param_server)
dynamic_model2 = SingleTrackModel(param_server)

behavior_model3 = BehaviorIDMClassic(param_server)
execution_model3 = ExecutionModelInterpolate(param_server)
dynamic_model3 = SingleTrackModel(param_server)

# Map Definition
xodr_parser = XodrParser("modules/runtime/tests/data/three_way_plain.xodr")
map_interface = MapInterface()
map_interface.SetOpenDriveMap(xodr_parser.map)
world.SetMap(map_interface)

# Agent Definition
agent_2d_shape = CarLimousine()
init_state = np.array([0, -20, -2, 0, 43/3.6])
goal_polygon = Polygon2d([0, 0, 0],[Point2d(-1,-1),Point2d(-1,1),Point2d(1,1), Point2d(1,-1)])
goal_polygon = goal_polygon.Translate(Point2d(30, -2))
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
init_state2 = np.array([0, 30, 2, -3.14, 50/3.6])
agent_params2 = param_server.addChild("agent2")
goal_polygon = Polygon2d([0, 0, 0], [Point2d(-1,-1),Point2d(-1,1),Point2d(1,1), Point2d(1,-1)])
goal_polygon = goal_polygon.Translate(Point2d(-30, 2))
agent2 = Agent(init_state2,
               behavior_model2,
               dynamic_model2,
               execution_model2,
               agent_2d_shape2,
               agent_params2,
               GoalDefinitionPolygon(goal_polygon),
               map_interface)
world.AddAgent(agent2)

agent_2d_shape3 = CarLimousine()
init_state3 = np.array([0, -30, -2, -3.14, 50/3.6])
agent_params3 = param_server.addChild("agent3")
goal_polygon = Polygon2d([0, 0, 0], [Point2d(-1,-1),Point2d(-1,1),Point2d(1,1), Point2d(1,-1)])
goal_polygon = goal_polygon.Translate(Point2d(-2, -30))
agent3 = Agent(init_state3,
               behavior_model3,
               dynamic_model3,
               execution_model3,
               agent_2d_shape3,
               agent_params3,
               GoalDefinitionPolygon(goal_polygon),
               map_interface)
world.AddAgent(agent3)


# viewer
viewer = MPViewer(params=param_server, use_world_bounds=True)

# World Simulation
sim_step_time = param_server["simulation"]["step_time",
                                           "Step-time used in simulation",
                                           0.2]
sim_real_time_factor = param_server["simulation"]["real_time_factor",
                                                  "execution in real-time or faster",
                                                  1]

for _ in range(0, 20):
  world.Step(sim_step_time)
  viewer.clear()
  viewer.drawWorld(world)
  viewer.drawRoadCorridor(agent3.road_corridor, color="green")

  # viewer.drawRoadCorridor(agent2.road_corridor, color="blue")
  viewer.show(block=False)
  time.sleep(sim_step_time/sim_real_time_factor)

param_server.save(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                  "params",
                  "od8_const_vel_one_agent_written.json"))