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
from bark.world.goal_definition import GoalDefinition, GoalDefinitionPolygon
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
    # TODO(@hart): Change this. Pass the right branch of the parameter server
    self._local_params = \
      self._params["Scenario"]["Generation"]["DeterministicScenarioGeneration"]
    
    self._map_file_name = self._local_params["MapFilename",
     "Path to the open drive map", 
     "modules/runtime/tests/data/Crossing8Course.xodr"]
    self._json_converter = ModelJsonConversion()

    self._agent_params = self._local_params["VehicleModel",
      "How to model the agent",
      self._json_converter.agent_to_json(self.default_agent_model())]

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
    scenario.agent_list = []
    for agent_json in self._local_params["Agents"]:
      agent_json["map_interface"] = world.map
      agent = self._json_converter.agent_from_json(agent_json)
      scenario.agent_list.append(agent)
    scenario._eval_agent_ids = [self._local_params["EgoAgentId",
                                "ID of the ego-agent",
                                0]]
    return scenario

  def default_agent_model(self):
    param_server = ParameterServer()
    behavior_model = BehaviorConstantVelocity(param_server)
    execution_model = ExecutionModelInterpolate(param_server)
    dynamic_model = SingleTrackModel()
    map_interface = MapInterface()
    agent_2d_shape = CarLimousine()
    init_state = np.array([0, 0, 0, 0, 0])
    goal_polygon = Polygon2d([0, 0, 0],
                             [Point2d(-1,-1),
                              Point2d(-1,1),
                              Point2d(1,1),
                              Point2d(1,-1)])
    # NOTE(@all): cannot add goal definition here since we do not have a map
    # goal_definition = GoalDefinitionPolygon(goal_polygon)
    # will be added in loop after world has been added
    agent_default = Agent(init_state,
                          behavior_model,
                          dynamic_model,
                          execution_model,
                          agent_2d_shape,
                          param_server)
    return agent_default