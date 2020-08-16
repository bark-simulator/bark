# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import numpy as np
import time
import os
from modules.runtime.commons.parameters import ParameterServer
from modules.runtime.viewer.pygame_viewer import PygameViewer
from modules.runtime.commons.xodr_parser import XodrParser
from bark.models.behavior import BehaviorConstantVelocity
from bark.models.execution import ExecutionModelInterpolate
from bark.models.dynamic import SingleTrackModel
from bark.world import World
from bark.world.goal_definition import GoalDefinitionPolygon
from bark.world.agent import Agent
from bark.world.map import MapInterface
from bark.geometry.standard_shapes import CarLimousine
from bark.geometry import Point2d, Polygon2d

from bark.world.evaluation import EvaluatorRss


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

behavior_model3 = BehaviorConstantVelocity(param_server)
execution_model3 = ExecutionModelInterpolate(param_server)
dynamic_model3 = SingleTrackModel(param_server)

map_path="modules/runtime/tests/data/centered_city_highway_straight.xodr"

# Map Definition
xodr_parser = XodrParser(map_path)
map_interface = MapInterface()
map_interface.SetOpenDriveMap(xodr_parser.map)
world.SetMap(map_interface)

# Agent Definition
agent_2d_shape = CarLimousine()
init_state = np.array([0, 1.8, -125, 0, 10])
goal_polygon = Polygon2d([0, 0, 0],[Point2d(-1,-1),Point2d(-1,1),Point2d(1,1), Point2d(1,-1)])
goal_polygon = goal_polygon.Translate(Point2d(1.8,120))
agent_params = param_server.addChild("agent")
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
init_state2 = np.array([0, 1.8, -50, 0, 7])
agent_params2 = param_server.addChild("agent")
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
goal_polygon3 = Polygon2d([0, 0, 0],[Point2d(-1,-1),Point2d(-1,1),Point2d(1,1), Point2d(1,-1)])
goal_polygon3 = goal_polygon3.Translate(Point2d(-1,120))
init_state3 = np.array([0, -1, -120, 0, 7])
agent_params3 = param_server.addChild("agent")
agent3 = Agent(init_state3,
               behavior_model3,
               dynamic_model3,
               execution_model3,
               agent_2d_shape3,
               agent_params3,
               GoalDefinitionPolygon(goal_polygon3),
               map_interface)
world.AddAgent(agent3)

# viewer
viewer = PygameViewer(params=param_server, follow_agent_id=agent1.id,x_range=[-60,60],y_range=[-60,60])

# World Simulation
sim_step_time = param_server["simulation"]["step_time",
                                           "Step-time used in simulation",
                                           1]
sim_real_time_factor = param_server["simulation"]["real_time_factor",
                                                  "execution in real-time or faster",
                                                  1]
e = EvaluatorRss(agent1.id, map_path, 
                 [3.5, -8., -4., -3., 0.2, -0.8, 0.1, 1.],
                 {1: [3.5, -8., -4., -3., 0.2, -0.8, 0.1, 1.], 
                 2: [5, -8., -4., -3., 0.2, -0.8, 0.1, 1.]})

# EvaluatorRss is accuarate only after stepping bark world
world.Step(0.5)

for _ in range(0, 30):
  viewer.clear()
  world.Step(sim_step_time)
  print(e.Evaluate(world),e.PairwiseEvaluate(world),e.PairwiseDirectionalEvaluate(world))
  viewer.drawWorld(world)
  viewer.drawSafetyResponses(world,agent1.id,e.PairwiseEvaluate(world))
  viewer.show(block=False)
  time.sleep(sim_step_time/sim_real_time_factor)

param_server.save(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                  "params",
                  "city_highway_straight.json"))
