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
from bark.world.goal_definition import GoalDefinitionPolygon, GoalDefinitionStateLimits, GoalDefinitionSequential
from bark.models.dynamic import *
from bark.models.execution import *
from bark.geometry import *
from bark.geometry.standard_shapes import *
from bark.world.evaluation import *
from modules.runtime.commons.parameters import ParameterServer


class EvaluationTests(unittest.TestCase):
  def test_one_agent_at_goal_polygon(self):
    param_server = ParameterServer()
    # Model Definition
    behavior_model = BehaviorConstantVelocity(param_server)
    execution_model = ExecutionModelInterpolate(param_server)
    dynamic_model = SingleTrackModel(param_server)

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

  def test_one_agent_at_goal_state_limits(self):
    param_server = ParameterServer()
    # Model Definition
    behavior_model = BehaviorConstantVelocity(param_server)
    execution_model = ExecutionModelInterpolate(param_server)
    dynamic_model = SingleTrackModel(param_server)

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
                GoalDefinitionStateLimits(goal_polygon, (3.14*3.0/4.0-0.08, 3.14*3.0/4.0+0.08)),
                  None)

    world = World(param_server)
    world.add_agent(agent)
    evaluator = EvaluatorGoalReached(agent.id)
    world.add_evaluator("success", evaluator)


    info = world.evaluate()
    self.assertEqual(info["success"], True)

  def test_one_agent_at_goal_sequential(self):
    param_server = ParameterServer()
    # Model Definition
    dynamic_model = SingleTrackModel(param_server)
    behavior_model = BehaviorMotionPrimitives(dynamic_model, param_server)
    idx = behavior_model.add_motion_primitive(np.array([1, 0]))
    behavior_model.action_to_behavior(idx)
    execution_model = ExecutionModelInterpolate(param_server)


    # Agent Definition
    agent_2d_shape = CarLimousine()
    init_state = np.array([0, 0, 0, 0, 0])
    agent_params = param_server.addChild("agent1")
    goal_frame = Polygon2d([0, 0, 0],
                             [Point2d(-1,-1),
                              Point2d(-1,1),
                              Point2d(1,1),
                              Point2d(1,-1)])

    goal_polygon1 = goal_frame.translate(Point2d(10, 0))
    goal_polygon2 = goal_frame.translate(Point2d(20, 0))
    goal_polygon3 = goal_frame.translate(Point2d(30, 0))

    goal_def1 = GoalDefinitionStateLimits(goal_polygon1, [-0.08, 0.08])
    goal_def2 = GoalDefinitionStateLimits(goal_polygon2, [-0.08, 0.08])
    goal_def3 = GoalDefinitionStateLimits(goal_polygon3, [-0.08, 0.08])

    goal_definition = GoalDefinitionSequential([goal_def1,
                                                goal_def2,
                                                goal_def3])

    self.assertEqual(len(goal_definition.sequential_goals),3)
    agent = Agent(init_state,
                behavior_model,
                dynamic_model,
                execution_model,
                agent_2d_shape,
                agent_params,
                goal_definition,
                  None)

    world = World(param_server)
    world.add_agent(agent)
    evaluator = EvaluatorGoalReached(agent.id)
    world.add_evaluator("success", evaluator)

    # just drive with the single motion primitive should be successful 
    for _ in range(0,1000):
        world.step(0.2)
        info = world.evaluate()
        if info["success"]:
            break
    
    self.assertEqual(info["success"], True)
    self.assertAlmostEqual(agent.state[int(StateDefinition.X_POSITION)], 30, delta=0.5)
      
        
if __name__ == '__main__':
  unittest.main()

