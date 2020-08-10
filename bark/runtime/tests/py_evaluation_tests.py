# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.


import numpy as np
import time
import unittest
from bark.runtime.commons.parameters import ParameterServer
from bark.runtime.viewer.matplotlib_viewer import MPViewer
from bark.runtime.commons.xodr_parser import XodrParser
from bark.core.models.behavior import BehaviorConstantAcceleration, \
  BehaviorMPContinuousActions
from bark.core.models.execution import ExecutionModelInterpolate
from bark.core.models.dynamic import SingleTrackModel, StateDefinition
from bark.core.world import World
from bark.core.world.goal_definition import GoalDefinitionPolygon, \
  GoalDefinitionStateLimits, GoalDefinitionSequential, \
    GoalDefinitionStateLimitsFrenet
from bark.core.world.agent import Agent
from bark.core.world.map import MapInterface
from bark.core.geometry.standard_shapes import CarLimousine
from bark.core.geometry import Point2d, Polygon2d, Line2d
from bark.core.world.evaluation import EvaluatorGoalReached, \
  EvaluatorCollisionEgoAgent, EvaluatorStepCount



class EvaluationTests(unittest.TestCase):
  def test_one_agent_at_goal_polygon(self):
    param_server = ParameterServer()
    # Model Definition
    behavior_model = BehaviorConstantAcceleration(param_server)
    execution_model = ExecutionModelInterpolate(param_server)
    dynamic_model = SingleTrackModel(param_server)

    # Agent Definition
    agent_2d_shape = CarLimousine()
    init_state = np.array([0, -191.789,-50.1725, 3.14*3.0/4.0, 150/3.6])
    agent_params = param_server.AddChild("agent1")
    goal_polygon = Polygon2d([0, 0, 0],
                             [Point2d(-4,-4),
                              Point2d(-4,4),
                              Point2d(4,4),
                              Point2d(4,-4)])
    goal_polygon = goal_polygon.Translate(Point2d(-191.789,-50.1725))

    agent = Agent(init_state,
                behavior_model,
                dynamic_model,
                execution_model,
                agent_2d_shape,
                agent_params,
                GoalDefinitionPolygon(goal_polygon),
                  None)

    world = World(param_server)
    world.AddAgent(agent)
    evaluator = EvaluatorGoalReached(agent.id)
    world.AddEvaluator("success", evaluator)


    info = world.Evaluate()
    self.assertEqual(info["success"], True)

  def test_one_agent_at_goal_state_limits(self):
    param_server = ParameterServer()
    # Model Definition
    behavior_model = BehaviorConstantAcceleration(param_server)
    execution_model = ExecutionModelInterpolate(param_server)
    dynamic_model = SingleTrackModel(param_server)

    # Agent Definition
    agent_2d_shape = CarLimousine()
    init_state = np.array([0, -191.789,-50.1725, 3.14*3.0/4.0, 150/3.6])
    agent_params = param_server.AddChild("agent1")
    goal_polygon = Polygon2d([0, 0, 0],
                             [Point2d(-1,-1),
                              Point2d(-1,1),
                              Point2d(1,1),
                              Point2d(1,-1)])
    goal_polygon = goal_polygon.Translate(Point2d(-191.789,-50.1725))

    agent = Agent(init_state,
                behavior_model,
                dynamic_model,
                execution_model,
                agent_2d_shape,
                agent_params,
                GoalDefinitionStateLimits(goal_polygon, (3.14*3.0/4.0-0.08, 3.14*3.0/4.0+0.08)),
                  None)

    world = World(param_server)
    world.AddAgent(agent)
    evaluator = EvaluatorGoalReached(agent.id)
    world.AddEvaluator("success", evaluator)


    info = world.Evaluate()
    self.assertEqual(info["success"], True)

  def test_one_agent_at_goal_sequential(self):
    param_server = ParameterServer()
    # Model Definition
    dynamic_model = SingleTrackModel(param_server)
    behavior_model = BehaviorMPContinuousActions(param_server)
    idx = behavior_model.AddMotionPrimitive(np.array([1, 0]))
    behavior_model.ActionToBehavior(idx)
    execution_model = ExecutionModelInterpolate(param_server)


    # Agent Definition
    agent_2d_shape = CarLimousine()
    init_state = np.array([0, 0, 0, 0, 0])
    agent_params = param_server.AddChild("agent1")
    goal_frame = Polygon2d([0, 0, 0],
                             [Point2d(-1,-1),
                              Point2d(-1,1),
                              Point2d(1,1),
                              Point2d(1,-1)])

    goal_polygon1 = goal_frame.Translate(Point2d(10, 0))
    goal_polygon2 = goal_frame.Translate(Point2d(20, 0))
    goal_polygon3 = goal_frame.Translate(Point2d(30, 0))

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
    world.AddAgent(agent)
    evaluator = EvaluatorGoalReached(agent.id)
    world.AddEvaluator("success", evaluator)

    # just drive with the single motion primitive should be successful 
    for _ in range(0,1000):
        world.Step(0.2)
        info = world.Evaluate()
        if info["success"]:
            break
    
    self.assertEqual(info["success"], True)
    self.assertAlmostEqual(agent.state[int(StateDefinition.X_POSITION)], 30, delta=0.5)
      

  def test_one_agent_at_goal_state_limits_frenet(self):
    param_server = ParameterServer()
    # Model Definition
    behavior_model = BehaviorConstantAcceleration(param_server)
    execution_model = ExecutionModelInterpolate(param_server)
    dynamic_model = SingleTrackModel(param_server)

    # Agent Definition
    agent_2d_shape = CarLimousine()
    agent_params = param_server.AddChild("agent1")

    center_line = Line2d()
    center_line.AddPoint(Point2d(5.0, 5.0))
    center_line.AddPoint(Point2d(10.0, 10.0))
    center_line.AddPoint(Point2d(20.0, 10.0))

    max_lateral_dist = (0.4,1)
    max_orientation_diff = (0.08, 0.1)
    velocity_range = (20.0, 25.0)
    goal_definition = GoalDefinitionStateLimitsFrenet(center_line,
                    max_lateral_dist, max_orientation_diff,
                    velocity_range)

    # not at goal x,y, others yes
    agent1 = Agent(np.array([0, 6, 8, 3.14/4.0 , velocity_range[0]]),
                behavior_model,
                dynamic_model,
                execution_model,
                agent_2d_shape,
                agent_params,
                goal_definition,
                  None)

    # at goal x,y and others
    agent2 = Agent(np.array([0, 5.0, 5.5, 3.14/4.0 , velocity_range[1]]),
                behavior_model,
                dynamic_model,
                execution_model,
                agent_2d_shape,
                agent_params,
                goal_definition,
                  None)

    # not at goal x,y,v yes but not orientation
    agent3 = Agent(np.array([0, 5, 5.5, 3.14/4.0+max_orientation_diff[1]+0.001 , 20]),
                behavior_model,
                dynamic_model,
                execution_model,
                agent_2d_shape,
                agent_params,
                goal_definition,
                  None)


    # not at goal x,y, orientation but not v
    agent4 = Agent(np.array([0, 5, 4.5, 3.14/4-max_orientation_diff[0], velocity_range[0]-0.01]),
                behavior_model,
                dynamic_model,
                execution_model,
                agent_2d_shape,
                agent_params,
                goal_definition,
                  None)

    # at goal x,y, at lateral limit 
    agent5 = Agent(np.array([0, 15, 10-max_lateral_dist[0]+0.05, 0, velocity_range[1]]),
                behavior_model,
                dynamic_model,
                execution_model,
                agent_2d_shape,
                agent_params,
                goal_definition,
                  None)

    # not at goal x,y slightly out of lateral limit 
    agent6 = Agent(np.array([0, 15, 10+max_lateral_dist[0]+0.05, 3.14/4+max_orientation_diff[0], velocity_range[0]]),
                behavior_model,
                dynamic_model,
                execution_model,
                agent_2d_shape,
                agent_params,
                goal_definition,
                  None)


    # not at goal x,y,v yes but not orientation
    agent7 = Agent(np.array([0, 5, 5.5, 3.14/4.0-max_orientation_diff[0]-0.001 , 20]),
                behavior_model,
                dynamic_model,
                execution_model,
                agent_2d_shape,
                agent_params,
                goal_definition,
                  None)


    world = World(param_server)
    world.AddAgent(agent1)
    world.AddAgent(agent2)
    world.AddAgent(agent3)
    world.AddAgent(agent4)
    world.AddAgent(agent5)
    world.AddAgent(agent6)
    world.AddAgent(agent7)

    evaluator1 = EvaluatorGoalReached(agent1.id)
    evaluator2 = EvaluatorGoalReached(agent2.id)
    evaluator3 = EvaluatorGoalReached(agent3.id)
    evaluator4 = EvaluatorGoalReached(agent4.id)
    evaluator5 = EvaluatorGoalReached(agent5.id)
    evaluator6 = EvaluatorGoalReached(agent6.id)
    evaluator7 = EvaluatorGoalReached(agent7.id)
    world.AddEvaluator("success1", evaluator1)
    world.AddEvaluator("success2", evaluator2)
    world.AddEvaluator("success3", evaluator3)
    world.AddEvaluator("success4", evaluator4)
    world.AddEvaluator("success5", evaluator5)
    world.AddEvaluator("success6", evaluator6)
    world.AddEvaluator("success7", evaluator7)


    info = world.Evaluate()
    self.assertEqual(info["success1"], False)
    self.assertEqual(info["success2"], True)
    self.assertEqual(info["success3"], False)
    self.assertEqual(info["success4"], False)
    self.assertEqual(info["success5"], True)
    self.assertEqual(info["success6"], False)
    self.assertEqual(info["success7"], False)
        
if __name__ == '__main__':
  unittest.main()

