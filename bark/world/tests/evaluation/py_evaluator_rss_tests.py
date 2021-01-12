# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import unittest
import pickle
import numpy as np

from bark.core.world import *
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
from bark.core.world.evaluation import EvaluatorRSS
from bark.core.commons import SetVerboseLevel
from bark.runtime.viewer import MPViewer


class TestAgent(Agent):
    """Derived Agent Class
    """

    def __init__(self, init_state, goal_polygon, map_interface, params):

        behavior_model = BehaviorConstantAcceleration(params)
        execution_model = ExecutionModelInterpolate(params)
        dynamic_model = SingleTrackModel(params)
        agent_2d_shape = CarLimousine()

        agent_params = params.AddChild("agent")
        super(TestAgent, self).__init__(init_state, behavior_model, dynamic_model,
                                        execution_model, agent_2d_shape, agent_params,
                                        GoalDefinitionPolygon(goal_polygon), map_interface)


class EvaluatorRSSTests(unittest.TestCase):

    @staticmethod
    def load_map(map):
        xodr_parser = XodrParser(map)
        map_interface = MapInterface()
        map_interface.SetOpenDriveMap(xodr_parser.map)
        return map_interface

    def test_longitude_highway_safe(self):
        """
        Checking Longitudinal Responses (true means safe)
        """

        params = ParameterServer()
        map = "bark/runtime/tests/data/city_highway_straight.xodr"
        params["EvaluatorRss"]["MapFilename"] = map

        map_interface = EvaluatorRSSTests.load_map(map)
        world = World(params)
        world.SetMap(map_interface)

        goal_polygon = Polygon2d(
            [0, 0, 0], [Point2d(-1, -1), Point2d(-1, 1), Point2d(1, 1), Point2d(1, -1)])
        goal_polygon = goal_polygon.Translate(Point2d(1.8, 120))

        # The safety distance seems more conservative than in the paper
        # Hard coded
        ego_state = np.array([0, 1.8, -114.9, np.pi/2, 10])
        other_state = np.array([0, 1.8, -72.95, np.pi/2, 7])

        ego = TestAgent(ego_state, goal_polygon, map_interface, params)
        other = TestAgent(other_state, goal_polygon, map_interface, params)

        world.AddAgent(ego)
        world.AddAgent(other)
        world.UpdateAgentRTree()

        viewer = MPViewer(params=params, use_world_bounds=True)
        viewer.drawWorld(world)
        viewer.show(block=False)

        evaluator_rss = EvaluatorRSS(ego.id, params)

        pw_directional_evaluation_return = evaluator_rss.PairwiseDirectionalEvaluate(
            world)
        self.assertEqual(True, pw_directional_evaluation_return[other.id][0])

    def test_longitude_highway_unsafe(self):
        """
        Checking Longitudinal Responses (true means safe)
        """

        params = ParameterServer()
        map = "bark/runtime/tests/data/city_highway_straight.xodr"
        params["EvaluatorRss"]["MapFilename"] = map

        map_interface = EvaluatorRSSTests.load_map(map)
        world = World(params)
        world.SetMap(map_interface)

        goal_polygon = Polygon2d(
            [0, 0, 0], [Point2d(-1, -1), Point2d(-1, 1), Point2d(1, 1), Point2d(1, -1)])
        goal_polygon = goal_polygon.Translate(Point2d(1.8, 120))

        # The safety distance seems more conservative than in the paper
        # Hard coded
        ego_state = np.array([0, 1.8, -60.0, np.pi/2, 10])
        other_state = np.array([0, 1.8, -68.0, np.pi/2, 10])

        ego = TestAgent(ego_state, goal_polygon, map_interface, params)
        other = TestAgent(other_state, goal_polygon, map_interface, params)

        world.AddAgent(ego)
        world.AddAgent(other)
        world.UpdateAgentRTree()

        viewer = MPViewer(params=params, use_world_bounds=True)
        viewer.drawWorld(world)
        viewer.show(block=False)

        evaluator_rss = EvaluatorRSS(ego.id, params)

        pw_directional_evaluation_return = evaluator_rss.PairwiseDirectionalEvaluate(
            world)
        self.assertEqual(False, pw_directional_evaluation_return[other.id][0])

    def test_lateral_highway_safe(self):
        """
        Checking Longitudinal Responses (true means safe)
        """

        params = ParameterServer()
        map = "bark/runtime/tests/data/city_highway_straight.xodr"
        params["EvaluatorRss"]["MapFilename"] = map

        map_interface = EvaluatorRSSTests.load_map(map)
        world = World(params)
        world.SetMap(map_interface)

        goal_polygon_1 = Polygon2d(
            [0, 0, 0], [Point2d(-1, -1), Point2d(-1, 1), Point2d(1, 1), Point2d(1, -1)])
        goal_polygon_1 = goal_polygon_1.Translate(Point2d(5.5, 120))

        goal_polygon_2 = Polygon2d(
            [0, 0, 0], [Point2d(-1, -1), Point2d(-1, 1), Point2d(1, 1), Point2d(1, -1)])
        goal_polygon_2 = goal_polygon_2.Translate(Point2d(1.8, 120))

        # Hard coded
        ego_state = np.array([0, 5.5, 10, np.pi/2, 10])  # straight north
        other_state = np.array([0, 1.8, 0, np.pi/2, 15])  # straight north

        ego = TestAgent(ego_state, goal_polygon_1, map_interface, params)
        other = TestAgent(other_state, goal_polygon_2, map_interface, params)

        world.AddAgent(ego)
        world.AddAgent(other)
        world.UpdateAgentRTree()

        viewer = MPViewer(params=params, use_world_bounds=True)
        viewer.drawWorld(world)
        viewer.show(block=False)

        evaluator_rss = EvaluatorRSS(ego.id, params)

        self.assertEqual(
            True, evaluator_rss.PairwiseDirectionalEvaluate(world)[other.id][1])

    def test_lateral_highway_unsafe(self):
        """
        Checking Lateral Responses (true means safe)
        """

        params = ParameterServer()
        map = "bark/runtime/tests/data/city_highway_straight.xodr"
        params["EvaluatorRss"]["MapFilename"] = map

        map_interface = EvaluatorRSSTests.load_map(map)
        world = World(params)
        world.SetMap(map_interface)

        goal_polygon_1 = Polygon2d(
            [0, 0, 0], [Point2d(-1, -1), Point2d(-1, 1), Point2d(1, 1), Point2d(1, -1)])
        goal_polygon_1 = goal_polygon_1.Translate(Point2d(5.5, 120))

        goal_polygon_2 = Polygon2d(
            [0, 0, 0], [Point2d(-1, -1), Point2d(-1, 1), Point2d(1, 1), Point2d(1, -1)])
        goal_polygon_2 = goal_polygon_2.Translate(Point2d(1.8, 120))

        # Hard coded
        ego_state = np.array([0, 5.0, 10, np.pi/2, 10])  # straight north
        other_state = np.array([0, 3.1, 0, np.pi/2, 10])  # straight north

        ego = TestAgent(ego_state, goal_polygon_1, map_interface, params)
        other = TestAgent(other_state, goal_polygon_2, map_interface, params)

        world.AddAgent(ego)
        world.AddAgent(other)
        world.UpdateAgentRTree()

        viewer = MPViewer(params=params, use_world_bounds=True)
        viewer.drawWorld(world)
        viewer.show(block=False)

        evaluator_rss = EvaluatorRSS(ego.id, params)

        self.assertEqual(
            False, evaluator_rss.PairwiseDirectionalEvaluate(world)[other.id][1])

    def test_lateral_merging_safe(self):
        """
        Checking Lateral Responses (true means safe)
        """

        params = ParameterServer()
        map = "bark/runtime/tests/data/DR_DEU_Merging_MT_v01_centered.xodr"
        params["EvaluatorRss"]["MapFilename"] = map

        map_interface = EvaluatorRSSTests.load_map(map)
        world = World(params)
        world.SetMap(map_interface)

        goal_polygon = Polygon2d(
            [0, 0, 0], [Point2d(-1, -1), Point2d(-1, 1), Point2d(1, 1), Point2d(1, -1)])
        goal_polygon = goal_polygon.Translate(Point2d(-15.4, 108.6))

        # Hard coded
        ego_state = np.array([0, 68.1, 108, -np.pi, 5])
        other_state = np.array([0, 64.1, 105, -np.pi, 5])

        ego = TestAgent(ego_state, goal_polygon, map_interface, params)
        other = TestAgent(other_state, goal_polygon, map_interface, params)

        world.AddAgent(ego)
        world.AddAgent(other)
        world.UpdateAgentRTree()

        viewer = MPViewer(params=params, use_world_bounds=True)
        viewer.drawWorld(world)
        viewer.show(block=False)

        evaluator_rss = EvaluatorRSS(ego.id, params)
        world.AddEvaluator("rss", evaluator_rss)

        pw_directional_evaluation_return = evaluator_rss.PairwiseDirectionalEvaluate(
            world)
        self.assertEqual(True, pw_directional_evaluation_return[other.id][1])

    def test_lateral_merging_unsafe(self):
        """
        Checking Lateral Responses (true means safe)
        """
        params = ParameterServer()
        map = "bark/runtime/tests/data/DR_DEU_Merging_MT_v01_centered.xodr"
        params["EvaluatorRss"]["MapFilename"] = map

        map_interface = EvaluatorRSSTests.load_map(map)
        world = World(params)
        world.SetMap(map_interface)

        goal_polygon = Polygon2d(
            [0, 0, 0], [Point2d(-1, -1), Point2d(-1, 1), Point2d(1, 1), Point2d(1, -1)])
        goal_polygon = goal_polygon.Translate(Point2d(-15.4, 108.6))

        # Hard coded
        ego_state = np.array([0, 62.8, 107.8, -np.pi+0.2, 5])
        other_state = np.array([0, 67.5, 105.3, -np.pi, 5])

        ego = TestAgent(ego_state, goal_polygon, map_interface, params)
        other = TestAgent(other_state, goal_polygon, map_interface, params)

        world.AddAgent(ego)
        world.AddAgent(other)
        world.UpdateAgentRTree()

        viewer = MPViewer(params=params, use_world_bounds=True)
        viewer.drawWorld(world)
        viewer.show(block=False)

        evaluator_rss = EvaluatorRSS(ego.id, params)
        world.AddEvaluator("rss", evaluator_rss)

        pw_directional_evaluation_return = evaluator_rss.PairwiseDirectionalEvaluate(
            world)
        self.assertEqual(False, pw_directional_evaluation_return[other.id][1])

    def test_relevant_agents(self):

        params = ParameterServer()
        map = "bark/runtime/tests/data/city_highway_straight.xodr"
        params["EvaluatorRss"]["MapFilename"] = map

        map_interface = EvaluatorRSSTests.load_map(map)
        world = World(params)
        world.SetMap(map_interface)

        goal_polygon_1 = Polygon2d(
            [0, 0, 0], [Point2d(-1, -1), Point2d(-1, 1), Point2d(1, 1), Point2d(1, -1)])
        goal_polygon_1 = goal_polygon_1.Translate(Point2d(5.5, 120))

        goal_polygon_2 = Polygon2d(
            [0, 0, 0], [Point2d(-1, -1), Point2d(-1, 1), Point2d(1, 1), Point2d(1, -1)])
        goal_polygon_2 = goal_polygon_2.Translate(Point2d(1.8, 120))

        ego_state = np.array([0, 5.5, 10, np.pi/2, 10])
        other_1_state = np.array([0, 1.8, -10, np.pi/2, 15])
        other_2_state = np.array([0, 1.8, -120, np.pi/2, 10])

        ego = TestAgent(ego_state, goal_polygon_1, map_interface, params)
        other_1 = TestAgent(other_1_state, goal_polygon_2,
                            map_interface, params)
        other_2 = TestAgent(other_2_state, goal_polygon_2,
                            map_interface, params)

        world.AddAgent(ego)
        world.AddAgent(other_1)
        world.AddAgent(other_2)

        viewer = MPViewer(params=params, use_world_bounds=True)
        viewer.drawWorld(world)
        viewer.show(block=False)

        evaluator_rss = EvaluatorRSS(ego.id, params)
        responses = evaluator_rss.PairwiseEvaluate(world)

        self.assertEqual(1, len(responses))
        self.assertTrue(responses[other_1.id])
        self.assertFalse(other_2.id in responses)


if __name__ == '__main__':
    SetVerboseLevel(4)
    unittest.main()
