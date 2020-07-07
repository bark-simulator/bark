# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

from bark.runtime.scenario import Scenario
from bark.runtime.scenario.scenario_generation import ScenarioGeneration
from bark.runtime.commons import ModelJsonConversion
from bark.core.world.agent import *
from bark.core.models.behavior import *
from bark.core.world import *
from bark.core.world.goal_definition import GoalDefinition, GoalDefinitionPolygon, GoalDefinitionSequential
from bark.core.world.map import *
from bark.core.models.dynamic import *
from bark.core.models.execution import *
from bark.core.geometry import *
from bark.core.geometry.standard_shapes import *
from bark.runtime.commons import ParameterServer
from bark.runtime.commons import XodrParser

import numpy as np
import math


class DeterministicDroneChallengeGeneration(ScenarioGeneration):
  def __init__(self, num_scenarios, params=None, random_seed=None):
    super(DeterministicDroneChallengeGeneration, self).__init__(params,
                                                                num_scenarios)
    self.initialize_params(params)

  def initialize_params(self, params):
    self._local_params = \
      self._params["Scenario"]["Generation"]["DeterministicDroneChallengeGeneration"]

    self.goal_frame_center = self._local_params["goal_frame"][
      "center_pose", "Center pose of the goal frames",
      [1, 0, 0]]
    self.goal_frame_points = self._local_params["goal_frame"]["polygon_points",
      "Points of the goal frame polygon",
      [[0., 0.],
       [2., 0.],
       [2., 0.5],
       [0., 0.5],
       [0., 0.]]]
    self.goal_frame_poses = self._local_params["goal_poses",
      "A list with x,y, theta specyfing the sequence of the goal frames",
      [[5, 10, -0.3],
      [10,15, -1.2],
      [20,10, -2.9],
      [15,2, -3.9]]]

    self._map_file_name = self._local_params["MapFilename",
     "Path to the open drive map", 
     "bark/runtime/tests/data/Crossing8Course.xodr"]
    self._json_converter = ModelJsonConversion()


  def create_scenarios(self, params, num_scenarios, random_seed):
    """ 
        see baseclass
    """
    scenario_list = []
    for scenario_idx in range(0, num_scenarios):
      scenario = self.create_single_scenario()     
      scenario_list.append(scenario)
    return scenario_list

  def _build_sequential_goal_definition(self):
    goal_list = []
    for goal_pose in self.goal_frame_poses:
        goal_polygon = Polygon2d(self.goal_frame_center,
                               np.array(self.goal_frame_points))
        goal_polygon = goal_polygon.Transform(goal_pose)
        goal_definition = GoalDefinitionPolygon(goal_polygon)
        goal_list.append(goal_definition)
    
    return GoalDefinitionSequential(goal_list)

  def create_single_scenario(self):
    scenario = Scenario(map_file_name=self._map_file_name,
                        json_params=self._params.ConvertToDict())
    scenario._map_interface = None
    world = scenario.GetWorldState()
    agent_list = []
    scenario._agent_list = []
    for agent_json_ in self._local_params["Agents"]:
      agent_json = agent_json_["VehicleModel"].copy()
      agent_json["map_interface"] = world.map
      goal_polygon = Polygon2d([0, 0, 0],
                               np.array(agent_json["goal"]["polygon_points"]))
      goal_polygon = goal_polygon.Translate(Point2d(agent_json["goal"]["center_pose"][0],
                                                    agent_json["goal"]["center_pose"][1]))

      agent_json["goal_definition"] = self._build_sequential_goal_definition()
      agent_state = np.array(agent_json["state"])
      if len(np.shape(agent_state)) > 1:
        agent_state = np.random.uniform(low=agent_state[:, 0],
                                                high=agent_state[:, 1])
      agent_json["state"] = agent_state.tolist()
      agent = self._json_converter.agent_from_json(agent_json,
                                                   param_server=self._local_params)
      agent.SetAgentId(agent_json["id"])
      scenario._agent_list.append(agent)
    scenario._eval_agent_ids = [self._local_params["EgoAgentId",
                                "ID of the ego-agent",
                                0]]
    return scenario