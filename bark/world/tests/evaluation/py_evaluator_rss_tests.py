# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import unittest
import numpy as np

from bark.core.world import *
from bark.core.world.evaluation.ltl import *
from bark.runtime.commons.parameters import ParameterServer
from bark.runtime.commons.xodr_parser import XodrParser
from bark.core.models.behavior import BehaviorConstantAcceleration
from bark.core.models.execution import ExecutionModelInterpolate
from bark.core.models.dynamic import SingleTrackModel
from bark.core.world import World
from bark.core.world.goal_definition import GoalDefinitionPolygon
from bark.core.world.agent import Agent
from bark.core.world.map import MapInterface
from bark.core.geometry.standard_shapes import CarLimousine
from bark.core.geometry import Point2d, Polygon2d
from bark.core.world.evaluation import EvaluatorRss

# General tests of the whole rss_interface
class EvaluatorRSSTests(unittest.TestCase):
  def setUp(self):
    param_server = ParameterServer()
    world = World(param_server)
    
    self.defaults = dict()
    self.defaults["world"] = world
    self.defaults["ego_behavior"] = BehaviorConstantAcceleration(
        param_server)
    self.defaults["ego_dynamic"] = SingleTrackModel(param_server)
    self.defaults["ego_execution"] = ExecutionModelInterpolate(
        param_server)
    self.defaults["ego_shape"] = CarLimousine()
    self.defaults["other_behavior"] = BehaviorConstantAcceleration(
        param_server)
    self.defaults["other_dynamic"] = SingleTrackModel(param_server)
    self.defaults["other_execution"] = ExecutionModelInterpolate(
        param_server)
    self.defaults["other_shape"] = CarLimousine()
    self.defaults["agent_params"] = param_server.addChild("agent")
    self.defaults["default_vehicle_dynamics"] = [
        1.7, -1.7, -1.69, -1.67, 0.2, -0.8, 0.1, 1.]

  @staticmethod
  def load_map(map):
    xodr_parser = XodrParser(map)
    map_interface = MapInterface()
    map_interface.SetOpenDriveMap(xodr_parser.map)
    return map_interface

  def test_longitude_ego_follow_other(self):
    map = "bark/runtime/tests/data/city_highway_straight_centered.xodr"
    map_interface = EvaluatorRSSTests.load_map(map)
    world = self.defaults["world"].Copy()
    world.SetMap(map_interface)

    goal_polygon = Polygon2d(
        [0, 0, 0], [Point2d(-1, -1), Point2d(-1, 1), Point2d(1, 1), Point2d(1, -1)])
    goal_polygon = goal_polygon.Translate(Point2d(1.8, 120))

    # The safety distance seems more conservative than in the paper
    # Hard coded
    ego_state = np.array([0, 1.8, -114.9, 0, 10])
    other_state = np.array([0, 1.8, -72.95, 0, 7])

    ego = Agent(
        ego_state,
        self.defaults["ego_behavior"],
        self.defaults["ego_dynamic"],
        self.defaults["ego_execution"],
        self.defaults["ego_shape"],
        self.defaults["agent_params"],
        GoalDefinitionPolygon(goal_polygon),
        map_interface)
    other = Agent(
        other_state,
        self.defaults["other_behavior"],
        self.defaults["other_dynamic"],
        self.defaults["other_execution"],
        self.defaults["other_shape"],
        self.defaults["agent_params"],
        GoalDefinitionPolygon(goal_polygon),
        map_interface)

    world.AddAgent(ego)
    world.AddAgent(other)

    evaluator_rss = EvaluatorRss(ego.id, map,
                                  self.defaults["default_vehicle_dynamics"],
                                  checking_relevent_range=1.5)
    world.Step(0.01)
    self.assertEqual(
        True, evaluator_rss.PairwiseDirectionalEvaluate(world)[other.id][0])
    world.Step(0.01)
    self.assertEqual(
        False, evaluator_rss.PairwiseDirectionalEvaluate(world)[other.id][0])

  def test_lateral_same_direction(self):
    map = "bark/runtime/tests/data/city_highway_straight_centered.xodr"
    map_interface = EvaluatorRSSTests.load_map(map)
    world = self.defaults["world"].Copy()
    world.SetMap(map_interface)

    goal_polygon_1 = Polygon2d(
        [0, 0, 0], [Point2d(-1, -1), Point2d(-1, 1), Point2d(1, 1), Point2d(1, -1)])
    goal_polygon_1 = goal_polygon_1.Translate(Point2d(5.5, 120))

    goal_polygon_2 = Polygon2d(
        [0, 0, 0], [Point2d(-1, -1), Point2d(-1, 1), Point2d(1, 1), Point2d(1, -1)])
    goal_polygon_2 = goal_polygon_2.Translate(Point2d(1.8, 120))

    # Hard coded
    ego_state = np.array([0, 5.5, 10, 0, 10])
    other_state = np.array([0, 1.8, -10, 0, 15])

    ego = Agent(
        ego_state,
        self.defaults["ego_behavior"],
        self.defaults["ego_dynamic"],
        self.defaults["ego_execution"],
        self.defaults["ego_shape"],
        self.defaults["agent_params"],
        GoalDefinitionPolygon(goal_polygon_1),
        map_interface)
    other = Agent(
        other_state,
        self.defaults["other_behavior"],
        self.defaults["other_dynamic"],
        self.defaults["other_execution"],
        self.defaults["other_shape"],
        self.defaults["agent_params"],
        GoalDefinitionPolygon(goal_polygon_2),
        map_interface)

    world.AddAgent(ego)
    world.AddAgent(other)

    evaluator_rss = EvaluatorRss(ego.id, map,
                                  self.defaults["default_vehicle_dynamics"])

    for _ in range(7):
      world.Step(1)
      self.assertEqual(
          True, evaluator_rss.PairwiseDirectionalEvaluate(world)[other.id][1])

  def test_lateral_merging(self):
    map = "bark/runtime/tests/data/DR_DEU_Merging_MT_v01_centered.xodr"
    map_interface = EvaluatorRSSTests.load_map(map)
    world = self.defaults["world"].Copy()
    world.SetMap(map_interface)

    goal_polygon = Polygon2d(
        [0, 0, 0], [Point2d(-1, -1), Point2d(-1, 1), Point2d(1, 1), Point2d(1, -1)])
    goal_polygon = goal_polygon.Translate(Point2d(-16, 108))

    # Hard coded
    ego_state = np.array([0, 68, 108, 0, 5])
    other_state = np.array([0, 64, 104, 0, 5])

    ego = Agent(
        ego_state,
        self.defaults["ego_behavior"],
        self.defaults["ego_dynamic"],
        self.defaults["ego_execution"],
        self.defaults["ego_shape"],
        self.defaults["agent_params"],
        GoalDefinitionPolygon(goal_polygon),
        map_interface)
    other = Agent(
        other_state,
        self.defaults["other_behavior"],
        self.defaults["other_dynamic"],
        self.defaults["other_execution"],
        self.defaults["other_shape"],
        self.defaults["agent_params"],
        GoalDefinitionPolygon(goal_polygon),
        map_interface)

    world.AddAgent(ego)
    world.AddAgent(other)

    evaluator_rss = EvaluatorRss(ego.id, map,
                                  self.defaults["default_vehicle_dynamics"])
    world.Step(1)
    self.assertEqual(
        True, evaluator_rss.PairwiseDirectionalEvaluate(world)[other.id][1])

    world.Step(1)
    self.assertEqual(
        False, evaluator_rss.PairwiseDirectionalEvaluate(world)[other.id][1])

  def test_relevent_agents(self):
    map = "bark/runtime/tests/data/city_highway_straight_centered.xodr"
    map_interface = EvaluatorRSSTests.load_map(map)
    world = self.defaults["world"].Copy()
    world.SetMap(map_interface)

    goal_polygon_1 = Polygon2d(
        [0, 0, 0], [Point2d(-1, -1), Point2d(-1, 1), Point2d(1, 1), Point2d(1, -1)])
    goal_polygon_1 = goal_polygon_1.Translate(Point2d(5.5, 120))

    goal_polygon_2 = Polygon2d(
        [0, 0, 0], [Point2d(-1, -1), Point2d(-1, 1), Point2d(1, 1), Point2d(1, -1)])
    goal_polygon_2 = goal_polygon_2.Translate(Point2d(1.8, 120))

    # Hard coded
    ego_state = np.array([0, 5.5, 10, 0, 10])
    other_1_state = np.array([0, 1.8, -10, 0, 15])
    other_2_state = np.array([0, 1.8, -120, 0, 10])

    ego = Agent(
        ego_state,
        self.defaults["ego_behavior"],
        self.defaults["ego_dynamic"],
        self.defaults["ego_execution"],
        self.defaults["ego_shape"],
        self.defaults["agent_params"],
        GoalDefinitionPolygon(goal_polygon_1),
        map_interface)
    other_1 = Agent(
        other_1_state,
        self.defaults["other_behavior"],
        self.defaults["other_dynamic"],
        self.defaults["other_execution"],
        self.defaults["other_shape"],
        self.defaults["agent_params"],
        GoalDefinitionPolygon(goal_polygon_2),
        map_interface)

    other_2_behavior = BehaviorConstantAcceleration(
        self.defaults["agent_params"])
    other_2_dynamic = SingleTrackModel(self.defaults["agent_params"])
    other_2_execution = ExecutionModelInterpolate(
        self.defaults["agent_params"])

    other_2 = Agent(
        other_2_state,
        other_2_behavior,
        other_2_dynamic,
        other_2_execution,
        self.defaults["other_shape"],
        self.defaults["agent_params"],
        GoalDefinitionPolygon(goal_polygon_2),
        map_interface)

    world.AddAgent(ego)
    world.AddAgent(other_1)
    world.AddAgent(other_2)

    evaluator_rss = EvaluatorRss(ego.id, map,
                                 self.defaults["default_vehicle_dynamics"])
    world.Step(1)
    responses = evaluator_rss.PairwiseEvaluate(world)
    
    self.assertEqual(1, len(responses)) # Test GetRelevantAgents
    self.assertTrue(responses[other_1.id])
    self.assertFalse(other_2.id in responses)

if __name__ == '__main__':
  unittest.main()
