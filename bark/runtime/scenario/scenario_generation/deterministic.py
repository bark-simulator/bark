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
from bark.core.world.goal_definition import GoalDefinitionPolygon, GoalDefinitionStateLimits, GoalDefinitionSequential
from bark.core.world.map import *
from bark.core.models.dynamic import *
from bark.core.models.execution import *
from bark.core.geometry import *
from bark.core.geometry.standard_shapes import *

import numpy as np


class DeterministicScenarioGeneration(ScenarioGeneration):
  def __init__(self, num_scenarios, params=None, random_seed=None):
    super(DeterministicScenarioGeneration, self).__init__(params,
                                                          num_scenarios)
    self.initialize_params(params)

  def initialize_params(self, params):
    self._local_params = \
      self._params["Scenario"]["Generation"]["DeterministicScenarioGeneration"]
    
    self._map_file_name = self._local_params["MapFilename",
     "Path to the open drive map", 
     "bark/runtime/tests/data/Crossing8Course.xodr"]
    self._json_converter = ModelJsonConversion()


  def create_scenarios(self, params, num_scenarios):
    """ 
        see baseclass
    """
    scenario_list = []
    for _ in range(0, num_scenarios):
      scenario = self.create_single_scenario()     
      scenario_list.append(scenario)
    return scenario_list

  def create_single_scenario(self):
    scenario = Scenario(map_file_name=self._map_file_name,
                        json_params=self._params.ConvertToDict())
    world = scenario.GetWorldState()
    agent_list = []
    scenario._agent_list = []
    for agent_json_ in self._local_params["Agents"]:
      agent_json = agent_json_["VehicleModel"].clone()
      agent_json["map_interface"] = world.map
      goal_polygon = Polygon2d([0, 0, 0],
                               np.array(agent_json["goal"]["polygon_points"]))
      goal_polygon = goal_polygon.Translate(Point2d(agent_json["goal"]["center_pose"][0],
                                                    agent_json["goal"]["center_pose"][1]))

      sequential_goals = []
      goal = GoalDefinitionPolygon(goal_polygon)
      if "goal_type" in agent_json["goal"]:
        goal_type = agent_json["goal"]["goal_type"]
        if goal_type == "GoalDefinitionStateLimits":
          goal = GoalDefinitionStateLimits(goal_polygon, (1.49, 1.65))

      # state_limit_goal = GoalDefinitionStateLimits(goal_polygon, (1.49, 1.65))
      for _ in range(self._local_params["goal"]["num_reached", "num", 5]):
        sequential_goals.append(goal)
      sequential_goal = GoalDefinitionSequential(sequential_goals)
      agent_json["goal_definition"] = sequential_goal

      agent_state = np.array(agent_json["state"])
      if len(np.shape(agent_state)) > 1:
        agent_state = np.random.uniform(low=agent_state[:, 0],
                                        high=agent_state[:, 1])
      agent_json["state"] = agent_state.tolist()
      agent = self._json_converter.agent_from_json(agent_json,
                                                   param_server=self._params)
      agent.SetAgentId(agent_json["id"])
      scenario._agent_list.append(agent)
    
    # TODO(@hart): this could be mult. agents
    scenario._eval_agent_ids = self._local_params["controlled_ids",
                                "IDs of agents to be controlled. ",
                                [0]]
    return scenario