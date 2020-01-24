# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

from modules.runtime.scenario.scenario import Scenario
from modules.runtime.scenario.scenario_generation.scenario_generation \
  import ScenarioGeneration
from modules.runtime.scenario.scenario_generation.model_json_conversion \
  import ModelConversion
from modules.runtime.scenario.scenario_generation.property_readers.property_readers import *
from bark.world.agent import *
from bark.models.behavior import *
from bark.world import *
from bark.world.goal_definition import GoalDefinition, GoalDefinitionPolygon, GoalDefinitionStateLimits
from bark.world.map import *
from bark.models.dynamic import *
from bark.models.execution import *
from bark.geometry import Polygon, Point2d
from bark.geometry.standard_shapes import *
from modules.runtime.commons.parameters import ParameterServer
from modules.runtime.commons.xodr_parser import XodrParser

import numpy as np
import math


class PropertyBasedScenarioGeneration(ScenarioGeneration):
  def __init__(self, num_scenarios, params=None):
    super(UniformVehicleDistribution, self).__init__(params,
                                                      num_scenarios,
                                                      random_seed)
    self.initialize_params(params)

  def initialize_params(self, params):
    params_temp = \
      self._params["Scenario"]["Generation"]["UniformVehicleDistribution"]
    self._map_file_name = params_temp["MapFilename",
      "Path to the open drive map", 
      "modules/runtime/tests/data/city_highway_straight.xodr",    ]
    self._random_seed = params_temp["RandomSeed", "Random seed used for sampling", 1000]
    self._sinks_sources = params_temp["SinksSources", "Random seed used for sampling", [{
      "SourceSink": (509, 509),
      "ConfigAgentStatesGeometries": {"type": "UniformDistribution"},
      "ConfigBehaviorModels": {},
      "ConfigExecutionModels": {},
      "ConfigDynamicModels": {},
    }]
    ]
    json_converter = ModelJsonConversion()
    self._agent_params = params_temp["DefaultVehicleModel",
      "How to model the other agents",
      json_converter.agent_to_json(self.default_agent_model())]
    if not isinstance(self._agent_params, dict):
        self._agent_params = self._agent_params.convert_to_dict()
    np.random.seed(self._random_seed)
    self.params = params

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
    agent_list_sources_sinks = {}
    sink_source_default_param_configs = []
    # Loop through each source sink config and create agents step by step
    # based on config sets for this source sink setting
    for idx, sink_source_config in enumerate(self._sinks_sources):
      road_corridor = self.get_road_corridor_from_source_sink(sink_source)
      
      #1) create agent states and geometries
      config_return, default_params_state_geomtry = 
        PropertyBasedScenarioGeneration.eval_configuration(
                              sink_source_config, "ConfigAgentStatesGeometries",
                              road_corridor)
      agent_states, agent_geometries, kwargs_dict = config_return
      # collect default parameters of this config
      sink_source_config["ConfigAgentStatesGeometries"] = default_params_state_geomtry

      #2) create behavior, execution and dynamic models
      config_return, default_params_behavior = 
        PropertyBasedScenarioGeneration.eval_configuration(
                              sink_source_config, "ConfigBehaviorModels",
                              road_corridor)
  
  @staticmethod
  def eval_configuration(sink_source_config, config_type, *args):
    eval_config = sink_source_config[config_type]
    eval_config_type = agent_state_geometry_config["type"]
    param_config = ParameterServer(json = eval_config)
    config_return = eval("{}({})".format(
        eval_config_type, ", ".join(args)))
    return config_return, default_param_config

  @staticmethod
  def get_road_corridor_from_source_sink(source_sink_properties):
    # generate road corridor between source and sink
    goal_polygon = Polygon2d([0, 0, 0],
                                  [Point2d(-1,0),
                                  Point2d(-1,1),
                                  Point2d(1,1),
                                  Point2d(1,0)])
    goal_polygon = goal_polygon.translate(Point2d(sink[0],
                                                      sink[1]))
    road_corridor = map_interface.GenerateRoadCorridor(
        Point2d(source[0], source[1]),
        goal_polygon)

    if not road_corridor:
      raise ValueError("No road corridor found betwen source {} \
                 and sink {}".format(source, sink) )

    return road_corridor

  def _parse_source_sink_properties(source_sink_properties):
    
