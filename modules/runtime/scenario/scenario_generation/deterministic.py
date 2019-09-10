# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

from modules.runtime.scenario.scenario import Scenario
from modules.runtime.scenario.scenario_generation.scenario_generation import ScenarioGeneration
from modules.runtime.scenario.scenario_generation.model_json_conversion import ModelJsonConversion
from bark.world.agent import *
from bark.models.behavior import *
from bark.world import *
from bark.world.goal_definition import GoalDefinition
from bark.world.map import *
from bark.models.dynamic import *
from bark.models.execution import *
from bark.geometry import *
from bark.geometry.standard_shapes import *
from modules.runtime.commons.parameters import ParameterServer
from modules.runtime.commons.xodr_parser import XodrParser

import numpy as np
import math


class DiscreteScenario(ScenarioGeneration):
  def __init__(self, num_scenarios, params=None, random_seed=None):
    super(DiscreteScenario, self).__init__(params,
                                           num_scenarios,
                                           random_seed)
    self.initialize_params(params)

  def initialize_params(self, params):
    params_temp = \
      self._params["Scenario"]["Generation"]["UniformVehicleDistribution"]
    self._map_file_name = params_temp["MapFilename",
     "Path to the open drive map", 
     "modules/runtime/tests/data/Crossing8Course.xodr"]
    self._ego_goal = \
      params_temp["EgoGoal",
      "The center of the ego agent's goal region polygon",
      [-191.789,-50.1725]]
    json_converter = ModelJsonConversion()
    self._agent_params = params_temp["VehicleModel", "How to model the agent", \
          json_converter.agent_to_json(self.default_agent_model())]
    if not isinstance(self._agent_params, dict):
        self._agent_params = self._agent_params.convert_to_dict()


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
    scenario = Scenario(map_file_name=self._map_file_name, json_params=self._params.convert_to_dict())
    world = scenario.get_world_state()
    agent_list = []
    """
    for idx, source in enumerate(self._others_source):
      connecting_center_line, s_start, s_end, _, lane_id_end = \
                self._center_line_between_source_and_sink( world.map,  source, self._others_sink[idx])
      goal_polygon = Polygon2d([0, 0, 0],[Point2d(-1.5,0),Point2d(-1.5,8),Point2d(1.5,8), Point2d(1.5,0)]) # todo: orient goal polygon along road
      goal_polygon = goal_polygon.translate(Point2d(self._others_sink[idx][0], self._others_sink[idx][1]))
      goal_definition = GoalDefinition(goal_polygon)
      agent_list.extend( self.place_agents_along_linestring(world, connecting_center_line, s_start, s_end, \
                                                                        self.agent_params, goal_definition) )

    description=self.params.convert_to_dict()
    description["ScenarioGenerator"] = "DiscreteScenario"
    scenario.agent_list = agent_list

    # set ego agent spec
    num_agents = len(scenario.agent_list)
    ego_agent = scenario.agent_list[math.floor(num_agents/4)] # put the agent in the middle of list 
    
    goal_polygon = Polygon2d([0, 0, 0],[Point2d(-1.5,0),Point2d(-1.5,8),Point2d(1.5,8), Point2d(1.5,0)])
    goal_polygon = goal_polygon.translate(Point2d(self.ego_goal[0], self.ego_goal[1]))
    ego_agent.goal_definition = GoalDefinition(goal_polygon)
    scenario.eval_agent_ids = [ego_agent.id] # only one agent is ego in the middle of all other agents
    """
    return scenario

  def default_agent_model(self):
    param_server = ParameterServer()
    behavior_model = BehaviorConstantVelocity(param_server)
    execution_model = ExecutionModelInterpolate(param_server)
    dynamic_model = SingleTrackModel()
    map_interface = MapInterface()
    agent_2d_shape = CarLimousine()
    init_state = np.array([0, 0, 0, 0, 0])
    agent_default = Agent(init_state,
                behavior_model,
                dynamic_model,
                execution_model,
                agent_2d_shape,
                param_server)
    return agent_default



  



    


