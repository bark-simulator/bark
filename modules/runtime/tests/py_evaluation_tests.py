# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT


import unittest
import os
import numpy as np
from bark.world.agent import *
from bark.models.behavior import *
from bark.world import *
from bark.world.map import *
from bark.world.goal_definition import GoalDefinitionPolygon
from bark.models.dynamic import *
from bark.models.execution import *
from bark.geometry import *
from bark.geometry.standard_shapes import *
from bark.world.evaluation import *
from modules.runtime.commons.parameters import ParameterServer


class EvaluationTests(unittest.TestCase):
  def test_one_agent_at_goal(self):
    param_server = ParameterServer()
    # Model Definition
    behavior_model = BehaviorConstantVelocity(param_server)
    execution_model = ExecutionModelInterpolate(param_server)
    dynamic_model = SingleTrackModel(param_server)

    print("test")
    # Agent Definition
    agent_2d_shape = CarLimousine()
    init_state = np.array([0, -191.789,-50.1725, 3.14*3.0/4.0, 150/3.6])
    agent_params = param_server.addChild("agent1")
    goal_polygon = Polygon2d([0, 0, 0],
                             [Point2d(-1,-1),
                              Point2d(-1,1),
                              Point2d(1,1),
                              Point2d(1,-1)])
    goal_polygon = goal_polygon.translate(Point2d(-191.789,-50.1725))

    agent = Agent(init_state,
                behavior_model,
                dynamic_model,
                execution_model,
                agent_2d_shape,
                agent_params,
                GoalDefinitionPolygon(goal_polygon),
                  None)

    world = World(param_server)
    world.add_agent(agent)
    evaluator = EvaluatorGoalReached(agent.id)
    world.add_evaluator("success", evaluator)


    info = world.evaluate()
    self.assertEqual(info["success"], True)
      
        
if __name__ == '__main__':
  unittest.main()

