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
import aabbtree
from collections import defaultdict 


class ConfigurableScenarioGeneration(ScenarioGeneration):
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
      "Description": "south_to_north",
      "ConfigAgentStatesGeometries": {"type": "UniformDistribution"},
      "ConfigBehaviorModels": {},
      "ConfigExecutionModels": {},
      "ConfigDynamicModels": {}
    },
    {
      "SourceSink": (509, 509),
      "Description": "south_to_west",
      "ConfigAgentStatesGeometries": {"type": "UniformDistribution"},
      "ConfigBehaviorModels": {},
      "ConfigExecutionModels": {},
      "ConfigDynamicModels": {}
    }]
    ]
    self._conflict_resolutions = params_temp["ConflictResolution", "How are conflicts for overlapping \
              sources and sinks resolved", {"south_to_west/south_to_north" : 0.2:0.8}]
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
    collected_sources_sinks_agent_states_geometries = []
    collected_sources_sinks_default_param_configs = []

    # Loop through each source sink config and first only create state
    # and geometry information
    for idx, sink_source_config in enumerate(self._sinks_sources):
      road_corridor = self.get_road_corridor_from_source_sink(sink_source)
      
      #1) create agent states and geometries for this source
      config_return, default_params_state_geometry = 
        PropertyBasedScenarioGeneration.eval_configuration(
                              sink_source_config, "ConfigAgentStatesGeometries",
                              road_corridor)
      agent_states, agent_geometries, kwargs_dict = config_return
      # collect default parameters of this config
      sink_source_config["ConfigAgentStatesGeometries"] = default_params_state_geometry
      collected_sources_sinks_agent_states_geometries.append(tuple(agent_states, agent_geometries))

    #2 remove overlapping agent states from different sources and sinks
    collected_sources_sinks_agent_states_geometries = 
            self.resolve_overlaps_in_sources_sinks_agents(collected_sources_sinks_agent_states_geometries)

    agent_list = []
    for idx, agent_states_geometries in enumerate(
                    collected_sources_sinks_agent_states_geometries):
      agent_states = agent_states_geometries[0]
      agent_geometries = agent_states_geometries[1]

      #3) create behavior, execution and dynamic models
      config_return, default_params_behavior = 
        PropertyBasedScenarioGeneration.eval_configuration(
                              sink_source_config, "ConfigBehaviorModels",
                              road_corridor, agent_states)
      behavior_models = config_return
      sink_source_config["ConfigBehaviorModels"] = default_params_behavior

      config_return, default_params_execution = 
        PropertyBasedScenarioGeneration.eval_configuration(
                              sink_source_config, "ConfigExecutionModels",
                              road_corridor, agent_states)
      execution_models = config_return
      sink_source_config["ConfigExecutionModels"] = default_params_execution

      config_return, default_params_dynamic = 
        PropertyBasedScenarioGeneration.eval_configuration(
                              sink_source_config, "ConfigDynamicModels",
                              road_corridor, agent_states)
      dynamic_models = config_return
      sink_source_config["ConfigDynamicModels"] = default_params_dynamic

      #4 create goal definitions and controlled agents
      config_return, default_params_goals = 
        PropertyBasedScenarioGeneration.eval_configuration(
                              sink_source_config, "ConfigGoalDefinitions",
                              road_corridor, agent_states)
      goal_definitions = config_return
      sink_source_config["ConfigGoalDefinitions"] = default_params_goals

      config_return, default_params_controlled_agents = 
        PropertyBasedScenarioGeneration.eval_configuration(
                              sink_source_config, "ConfigControlledAgents",
                              road_corridor, agent_states)
      controlled_agent_ids = config_return
      sink_source_config["ConfigControlledAgents"] = default_params_controlled_agents

      #5 Build all agents for this source config
      sink_source_agents = self.create_source_config_agents(agent_states,
                      agent_geometries, behavior_models, execution_models,
                      dynamic_models, goal_definitions, controlled_agent_ids,
                      world)

      agent_list.extend(sink_source_agents)
      collected_sources_sinks_default_param_configs.append(sink_source_config)

      return agent_list

  def resolve_overlaps_in_sources_sinks_agents(self, 
                  collected_sources_sinks_agent_states_geometries):
    # create kdtree for fast distance lookup between agents
    tree = aabbtree.AABBTree()
    for source_sink_idx, states_geometries in enumerate(
            collected_sources_sinks_agent_states_geometries):
      agent_states = states_geometries[0]
      agent_geometries = states_geometries[1]
      for agent_idx, agent_state in enumerate(agent_states):
        agent_translated_polygon = agent_geometries[agent_idx].translate(
                                                      Point2d(agent_state[0],
                                                            agent_state[1]))
        tmp = agent_translated_polygon.bounding_box
        bb = [(tmp[0].x, tmp[0].y), (tmp[1].x, tmp[1].y)]
        tree.add(bb, (source_sink_idx, agent_idx))

    # check for all agents in a source sink config collision with
    # agents in other sources sinks and track the collisions
    collisions = defaultdict(list)
    for source_sink_idx, states_geometries in enumerate(
            collected_sources_sinks_agent_states_geometries):
        agent_states = states_geometries[0]
        agent_geometries = states_geometries[1]
        for agent_idx, agent_state in enumerate(agent_states):
          agent_translated_polygon = agent_geometries[agent_idx].translate(
                                                        Point2d(agent_state[0],
                                                              agent_state[1]))
          tmp = agent_translated_polygon.bounding_box
          bb = [(tmp[0].x, tmp[0].y), (tmp[1].x, tmp[1].y)]
          overlaps = tree.overlap_values(bb)
          for overlap in overlaps:
            if source_sink_idx == overlap[0]:
              raise ValueError("Something went wrong. \
                   We have colliding agent within one source sink configuration")
            key1 = "{}{}".format(source_sink_idx, overlap[0])
            key2 = "{}{}".format(overlap[0], source_sink_idx, )
            if key1 in collisions:
              pairwise_collisions = collisions[key1]
            elif key2 in collisions:
              pairwise_collisions = collisions[key2]
            else:
              pairwise_collisions = []
              
            found = False
            for collision in pairwise_collisions:
              # exclude the case where this collision was already detected in a previous
              # run through the loop
              agent_desc1 = collision[0]
              if overlap[0] == agent_desc1[0] and overlap[1] == agent_desc1[1]:
                found = True
            if not found:
              agent_geometry_other = collected_sources_sinks_agent_states_geometries[overlap[0]][overlap[1]]
              agent_state_other = collected_sources_sinks_agent_states_geometries[overlap[0]][overlap[1]]
              agent_translated_polygon_other = agent_geometry_other.translate(
                                              Point2d(agent_state_other[0],
                                                    agent_state_other[1]))
              if collides(agent_translated_polygon, agent_translated_polygon_other):
                pairwise_collisions.append(((source_sink_idx, agent_idx), overlap))

    # use conflict resolution scheme to remove collisions
    for key, pairwise_collisions in collisions.items():
      # important check if all pairs still exist


  def create_source_config_agents(self, agent_states, agent_geometries, 
                        behavior_models, execution_models, dynamic_models
                        goal_definitions, controlled_agent_ids, world)
    num_agents = len(agent_states)
    if any(len(lst) != num_agents for list in [
      agent_geometries, behavior_models, execution_models, dynamic_models, goal_definitions, controlled_agent_ids]):
      raise ValueError("Config readers did not return equal sized of lists")
    agents = []
    for idx, agent_state in enumerate(agent_states):
      bark_agent = Agent( np.array(agent_state), 
                          behavior_models[idx], 
                          dynamic_models[idx],
                          execution_models[idx], 
                          agent_geometries[idx],
                          self._agent_params,
                          goal_definitions[idx],
                          world.map )

      agents.append(bark_agent)
    
    return agents

  
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
    
