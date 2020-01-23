# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

from modules.runtime.scenario.scenario import Scenario
from modules.runtime.scenario.scenario_generation.scenario_generation\
  import ScenarioGeneration
from modules.runtime.scenario.scenario_generation.model_json_conversion\
  import ModelJsonConversion
from bark.world.agent import *
from bark.models.behavior import *
from bark.world import *
from bark.world.goal_definition import GoalDefinition, GoalDefinitionPolygon, GoalDefinitionStateLimits, GoalDefinitionSequential
from bark.world.map import *
from bark.models.dynamic import *
from bark.models.execution import *
from bark.geometry import *
from bark.geometry.standard_shapes import *
from modules.runtime.commons.parameters import ParameterServer
from modules.runtime.commons.xodr_parser import XodrParser

import numpy as np
import math


class DeterministicScenarioGeneration(ScenarioGeneration):
  def __init__(self, num_scenarios, params=None, random_seed=None):
    super(DeterministicScenarioGeneration, self).__init__(params,
                                                          num_scenarios,
                                                          random_seed)
    self.initialize_params(params)

  def initialize_params(self, params):
    self._local_params = \
      self._params["Scenario"]["Generation"]["DeterministicScenarioGeneration"]
    
    self._map_file_name = self._local_params["MapFilename",
     "Path to the open drive map", 
     "modules/runtime/tests/data/Crossing8Course.xodr"]
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

  def create_single_scenario(self):
    scenario = Scenario(map_file_name=self._map_file_name,
                        json_params=self._params.convert_to_dict())
    world = scenario.get_world_state()
    agent_list = []
    scenario._agent_list = []
    for agent_json_ in self._local_params["Agents"]:
      agent_json = agent_json_["VehicleModel"].copy()
      agent_json["map_interface"] = world.map
      goal_polygon = Polygon2d([0, 0, 0],
                               np.array(agent_json["goal"]["polygon_points"]))
      goal_polygon = goal_polygon.translate(Point2d(agent_json["goal"]["center_pose"][0],
                                                    agent_json["goal"]["center_pose"][1]))

      sequential_goals = []
      # TODO(@ahrt): support other goals
      state_limit_goal = GoalDefinitionStateLimits(goal_polygon, (1.49, 1.65))
      # state_limit_goal = GoalDefinitionPolygon(goal_polygon)
      for _ in range(self._local_params["goal"]["num_reached", "num", 5]):
        sequential_goals.append(state_limit_goal)
      sequential_goal = GoalDefinitionSequential(sequential_goals)
      agent_json["goal_definition"] = sequential_goal

      agent_state = np.array(agent_json["state"])
      if len(np.shape(agent_state)) > 1:
        agent_state = np.random.uniform(low=agent_state[:, 0],
                                        high=agent_state[:, 1])
      agent_json["state"] = agent_state.tolist()
      agent = self._json_converter.agent_from_json(agent_json,
                                                   param_server=self._local_params)
      agent.set_agent_id(agent_json["id"])
      scenario._agent_list.append(agent)
    
    # TODO(@hart): this could be mult. agents
    scenario._eval_agent_ids = self._local_params["controlled_ids",
                                "IDs of agents to be controlled. ",
                                [0]]
    return scenario