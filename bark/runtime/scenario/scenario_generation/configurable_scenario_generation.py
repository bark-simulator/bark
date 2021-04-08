# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

from bark.runtime.scenario import Scenario
from bark.runtime.scenario.scenario_generation  import ScenarioGeneration
from bark.runtime.scenario.scenario_generation.config_readers import *
from bark.runtime.commons.parameters import ParameterServer

from bark.core.geometry import *
from bark.core.world.agent import Agent

import numpy as np
import math
import copy
import importlib
import aabbtree
from collections import defaultdict 

__CONFIG_READER_MODULES = []

def get_config_reader_modules():
  return __CONFIG_READER_MODULES

def add_config_reader_module(module_name):
  module = importlib.import_module(module_name)
  __CONFIG_READER_MODULES.append(module)


class ConfigurableScenarioGeneration(ScenarioGeneration):
  def __init__(self, num_scenarios, params=None, random_seed=1000):
    super(ConfigurableScenarioGeneration, self).__init__(params, num_scenarios, random_seed)

  def initialize_params(self, params):
    print (params["Scenario"])
    params_temp = \
      self._params["Scenario"]["Generation"]["ConfigurableScenarioGeneration"]
    self._map_file_name = params_temp["MapFilename",
      "Path to the open drive map", 
      "bark/runtime/tests/data/city_highway_straight.xodr", ]
    self._observer_model_params = params_temp[
      "ObserverModel",
      "World observer for the simulation.", {
        "Description": "world_observer",
        "ConfigObserverModel": {
          "Type": "ObserverModelNoneReader"
        }
    }]

    self._sinks_sources = params_temp["SinksSources", "Random seed used for sampling", [{
      "SourceSink": [[-1.057, -172.1965],  [-1.894, 14.1725] ],
      "Description": "left_lane",
      "ConfigAgentStatesGeometries": {"Type": "UniformVehicleDistribution", "LanePositions": [0]},
      "ConfigBehaviorModels": {"Type": "FixedBehaviorType", "ModelType" : "BehaviorIDMClassic", "ModelParams" :  {"BehaviorIDMClassic::MaxVelocity" : 60.0}},
      "ConfigExecutionModels": {"Type": "FixedExecutionType"},
      "ConfigDynamicModels": {"Type": "FixedDynamicType"},
      "ConfigGoalDefinitions": {"Type": "FixedGoalTypes"},
      "ConfigControlledAgents": {"Type": "NoneControlled"},
      "AgentParams" : {}
    },
    {
      "SourceSink": [[-1.057, -172.1965],  [-1.894, 14.1725] ],
      "Description": "right_lane",
      "ConfigAgentStatesGeometries": {"Type": "UniformVehicleDistribution", "LanePositions": [1]},
      "ConfigBehaviorModels": {"Type": "FixedBehaviorType", "ModelType" : "BehaviorIDMClassic", "ModelParams" :  {"BehaviorIDMClassic::MaxVelocity" : 30.0}},
      "ConfigExecutionModels": {"Type": "FixedExecutionType"},
      "ConfigDynamicModels": {"Type": "FixedDynamicType"},
      "ConfigGoalDefinitions": {"Type": "FixedGoalTypes"},
      "ConfigControlledAgents": {"Type": "RandomSingleAgent"},
      "AgentParams" : {}
    }
    ]
    ]

    self._conflict_resolutions = params_temp["ConflictResolution", "How are conflicts for overlapping \
              sources and sinks resolved", {"left_lane/right_lane" : (0.2, 0.8)}]
    self._random_state = np.random.RandomState(self._random_seed)

    # all parameter servers used by all bark class instances must be persisted
    # otherwise parameter server serialization fails
    self._sink_source_parameter_servers = defaultdict(list)

    self._sink_source_default_params = None

  def update_defaults_params(self):
    pass
    #  self._params["Scenario"]["Generation"]["ConfigurableScenarioGeneration"]["SinksSources"] = \
     #     self._sink_source_default_params

  def add_config_reader_parameter_servers(self, description, config_type, config_reader):
    
    self._sink_source_parameter_servers[config_type].append(
      {"Description": description,
       "ParameterServers" : config_reader.get_param_servers()}
    )

  def add_config_module_dir(self, dir):
    module = importlib.import_module(dir)
    self._imported_modules.append(module)

  def get_persisted_param_servers(self):
    return self._sink_source_parameter_servers

  def create_scenarios(self, params, num_scenarios):
    """ 
        see baseclass
    """
    scenario_list = []
    for idx in range(0, num_scenarios):
      self._current_scenario_idx = idx
      scenario = self.create_single_scenario()     
      scenario_list.append(scenario)
    self.update_defaults_params()
    return scenario_list

  def create_single_scenario(self):
    scenario = Scenario(map_file_name=self._map_file_name,
                        json_params=self._params.ConvertToDict())
    world = scenario.GetWorldState()
    collected_sources_sinks_agent_states_geometries = []
    collected_sources_sinks_default_param_configs = []

    # Loop through each source sink config and first only create state
    # and geometry information
    road_corridors = []
    kwargs_agent_states_geometry = []
    sink_source_default_params = []

    for idx, sink_source_config in enumerate(self._sinks_sources):
      road_corridor = self.get_road_corridor_from_source_sink(sink_source_config, world.map)
      road_corridors.append(road_corridor)
  
      #1) create agent states and geometries for this source
      args = [road_corridor]
      agent_states, agent_geometries, kwargs_dict, default_params_state_geometry = \
        self.eval_configuration( sink_source_config, "ConfigAgentStatesGeometries",
                              args, {})
      kwargs_agent_states_geometry.append(kwargs_dict)

      # collect default parameters of this config
      #sink_source_default_params.append(sink_source_config)
      #sink_source_default_params[idx]["ConfigAgentStatesGeometries"] = default_params_state_geometry.ConvertToDict()
      collected_sources_sinks_agent_states_geometries.append((agent_states, agent_geometries))

    agent_list = []
    controlled_agent_ids_all = []
    for idx, agent_states_geometries in enumerate(
                    collected_sources_sinks_agent_states_geometries):
      sink_source_config = self._sinks_sources[idx]
      agent_states = agent_states_geometries[0]
      agent_geometries = agent_states_geometries[1]
      road_corridor = road_corridors[idx]

      if(len(agent_states)== 0):
        continue

      #3) create behavior, execution and dynamic models
      args_list = [road_corridor, agent_states ]
      kwargs_dict = {**kwargs_agent_states_geometry[idx]}
      config_return, kwargs_dict_tmp, default_params_behavior = \
        self.eval_configuration(
                              sink_source_config, "ConfigBehaviorModels", 
                              args_list, kwargs_dict)
      behavior_models = config_return
      #sink_source_default_params[idx]["ConfigBehaviorModels"] = default_params_behavior.ConvertToDict()
      
      kwargs_dict = {**kwargs_dict, **kwargs_dict_tmp}
      config_return, kwargs_dict_tmp, default_params_execution = \
        self.eval_configuration(
                              sink_source_config, "ConfigExecutionModels",
                              args_list, kwargs_dict)
      execution_models = config_return
      #sink_source_default_params[idx]["ConfigExecutionModels"] = default_params_execution.ConvertToDict()
      kwargs_dict = {**kwargs_dict, **kwargs_dict_tmp}


      config_return, kwargs_dict_tmp, default_params_dynamic = \
        self.eval_configuration(
                              sink_source_config, "ConfigDynamicModels",
                              args_list, kwargs_dict)
      dynamic_models = config_return
      #sink_source_default_params[idx]["ConfigDynamicModels"] = default_params_dynamic.ConvertToDict()
      kwargs_dict = {**kwargs_dict, **kwargs_dict_tmp}

      #4 create goal definitions and controlled agents
      config_return, kwargs_dict_tmp, default_params_controlled_agents = \
        self.eval_configuration(
                              sink_source_config, "ConfigControlledAgents",
                              args_list, kwargs_dict)
      controlled_agent_ids = config_return
      controlled_agent_ids_all.extend(controlled_agent_ids)
      #sink_source_default_params[idx]["ConfigControlledAgents"] = default_params_controlled_agents.ConvertToDict()
      kwargs_dict = {**kwargs_dict, **kwargs_dict_tmp}

      args_list = [*args_list, controlled_agent_ids]
      config_return, kwargs_dict_tmp, default_params_goals = \
        self.eval_configuration(
                              sink_source_config, "ConfigGoalDefinitions", 
                              args_list, kwargs_dict)
      goal_definitions = config_return
      #sink_source_default_params[idx]["ConfigGoalDefinitions"] = default_params_goals.ConvertToDict()

      #5 Build all agents for this source config
      kwargs_dict = {**kwargs_dict, **kwargs_dict_tmp}
      agent_params = sink_source_config["AgentParams"]
      sink_source_agents = self.create_source_config_agents(agent_states,
                      agent_geometries, behavior_models, execution_models,
                      dynamic_models, goal_definitions, controlled_agent_ids,
                      world, agent_params)
      #sink_source_default_params[idx]["AgentParams"] = agent_params.ConvertToDict()

      self.update_road_corridors(sink_source_agents, road_corridor)
      agent_list.extend(sink_source_agents)
      #collected_sources_sinks_default_param_configs.append(sink_source_config)

    #self._sink_source_default_params = sink_source_default_params
    
    scenario._eval_agent_ids = [i for i, val in enumerate(controlled_agent_ids_all) if val] 
    scenario._agent_list = self.update_agent_ids(agent_list)
    
    # 6. set observer model for the world
    observer_model, _, _ = self.eval_configuration(
      self._observer_model_params, "ConfigObserverModel", [], {})
    scenario._observer_model = observer_model
    
    return scenario

  def update_road_corridors(self, sink_source_agents, road_corridor):
    for idx, agent in enumerate(sink_source_agents):
      agent.road_corridor = road_corridor

  def update_agent_ids(self, agent_list):
    for idx, agent in enumerate(agent_list):
      agent.SetAgentId(idx)
    return agent_list

  def resolve_overlaps_in_sources_sinks_agents(self, 
                  collected_sources_sinks_agent_states_geometries):

      collisions = ConfigurableScenarioGeneration.find_overlaps_in_sources_sinks_agents(
        collected_sources_sinks_agent_states_geometries
      )
      return self.delete_overlaps_in_sources_sinks_agents(
        collected_sources_sinks_agent_states_geometries,
        collisions
      )

  @staticmethod
  def find_overlaps_in_sources_sinks_agents( 
                  collected_sources_sinks_agent_states_geometries):
    # create aabbtree for fast distance lookup between agents
    tree = aabbtree.AABBTree()
    for source_sink_idx, states_geometries in enumerate(
            collected_sources_sinks_agent_states_geometries):
      agent_states = states_geometries[0]
      agent_geometries = states_geometries[1]
      for agent_idx, agent_state in enumerate(agent_states):
        agent_translated_polygon = agent_geometries[agent_idx].Transform(
                                                        [agent_state[1],
                                                        agent_state[2], 
                                                        agent_state[3]])
        tmp = agent_translated_polygon.bounding_box
        bb = aabbtree.AABB([(tmp[0].x(), tmp[1].x()), (tmp[0].y(), tmp[1].y())])
        tree.add(bb, (source_sink_idx, agent_idx))

    # check for all agents in a source sink config collision with
    # agents in other sources sinks and track the collisions
    collisions = defaultdict(list)
    for source_sink_idx, states_geometries in enumerate(
            collected_sources_sinks_agent_states_geometries):
        agent_states = states_geometries[0]
        agent_geometries = states_geometries[1]
        for agent_idx, agent_state in enumerate(agent_states):
          agent_translated_polygon = agent_geometries[agent_idx].Transform(
                                                        [agent_state[1],
                                                        agent_state[2], 
                                                        agent_state[3]])
          tmp = agent_translated_polygon.bounding_box
          bb = aabbtree.AABB([(tmp[0].x(), tmp[1].x()), ( tmp[0].y() , tmp[1].y())])
          overlaps = tree.overlap_values(bb)
          for overlap in overlaps:
            if source_sink_idx == overlap[0]:
              if agent_idx == overlap[1]:
                continue
              else:
                raise ValueError("Something went wrong. \
                   We have colliding agent within one source sink configuration")

            key1 = "{}-{}".format(source_sink_idx, overlap[0])
            key2 = "{}-{}".format(overlap[0], source_sink_idx, )
            key = None
            if key1 in collisions:
              key = key1
            elif key2 in collisions:
              key = key2
            else:
              key = key1

            pairwise_collisions = collisions[key]
            found = False
            for collision in pairwise_collisions:
              # exclude the case where this collision was already detected in a previous
              # run through the loop
              agent_desc1 = collision[0]
              if overlap[0] == agent_desc1[0] and overlap[1] == agent_desc1[1]:
                found = True
            if not found:
              agent_geometry_other = collected_sources_sinks_agent_states_geometries[overlap[0]][1][overlap[1]]
              agent_state_other = collected_sources_sinks_agent_states_geometries[overlap[0]][0][overlap[1]]
              agent_translated_polygon_other = agent_geometry_other.Transform(
                                                        [agent_state_other[1],
                                                        agent_state_other[2], 
                                                        agent_state_other[3]])
              if Collide(agent_translated_polygon, agent_translated_polygon_other):
                pairwise_collisions.append(((source_sink_idx, agent_idx), overlap))
            
                collisions[key] = pairwise_collisions
    
    return collisions

  def delete_overlaps_in_sources_sinks_agents(self, 
                  collected_sources_sinks_agent_states_geometries, collisions):
    # first find all agents to delete, then delete in go
    must_delete = defaultdict(list)
    np.random.seed(self._random_seed)
    for key, pairwise_collisions in collisions.items():
      # get the conflict resolution scheme defined using descriptions in src sink configs  "left_turn/straight": 0.2:0.8
      source_sink_idx_1 = key.split("-")[0]
      source_sink_idx_2 = key.split("-")[1]

      source_sink_desc_1 = self._sinks_sources[int(source_sink_idx_1)]["Description"]
      source_sink_desc_2 = self._sinks_sources[int(source_sink_idx_2)]["Description"]
         
      num_conflict_res1, num_conflict_res2, probablistic_conflict_resolution = \
         self.parse_conflict_resolution(source_sink_desc_1, source_sink_desc_2)

      if not probablistic_conflict_resolution:
        raise NotImplemented("Not implemented yet.")

      for pairwise_collision in pairwise_collisions:
        source_sink_idx_1 = pairwise_collision[0][0]
        source_sink_idx_2 = pairwise_collision[1][0]
        agent_idx_1 = pairwise_collision[0][1]
        agent_idx_2 = pairwise_collision[1][1]

        if probablistic_conflict_resolution:
          sample = np.random.rand()
          if 0 <= sample and sample <= num_conflict_res1:
            if not agent_idx_1 in must_delete[source_sink_idx_1]:
              must_delete[source_sink_idx_1].append(agent_idx_1)
          elif num_conflict_res1 < sample <= (num_conflict_res1+num_conflict_res2):
            if not agent_idx_2 in must_delete[source_sink_idx_2]:
              must_delete[source_sink_idx_2].append(agent_idx_2)
          else:
            if not agent_idx_1 in must_delete[source_sink_idx_1]:
              must_delete[source_sink_idx_1].append(agent_idx_1)
            if not agent_idx_2 in must_delete[source_sink_idx_2]:
              must_delete[source_sink_idx_2].append(agent_idx_2)

    for source_sink_idx, agent_deletions in must_delete.items():
      agent_list = collected_sources_sinks_agent_states_geometries[source_sink_idx]
      if not isinstance(agent_deletions, list):
        agent_deletions = [agent_deletions]
      for index in list(sorted(agent_deletions, reverse=True)):
        del agent_list[0][index]
        del agent_list[1][index]

    return collected_sources_sinks_agent_states_geometries

  def parse_conflict_resolution(self, description1, description_2):
    # find for two src sink configs the conflict resolution and parse it
    key = self.conflict_resolution_key_from_src_conf_desc(description1, description_2)
    key_reverse = self.conflict_resolution_key_from_src_conf_desc(description_2, description1)
    reversed = False
    if not key in self._conflict_resolutions:
      if key_reverse in self._conflict_resolutions:
        key = key_reverse
        reversed = True
      else:
        raise ValueError("Conflict resolution scheme for {} and {} not specified.".format(description1, description2))
    conflict_res_1 = self._conflict_resolutions[key][0]
    conflict_res_2 = self._conflict_resolutions[key][1]
    if reversed:
      tmp = conflict_res_1
      conflict_res_1 = conflict_res_2
      conflict_res_2 = tmp

    
    is_int_1 = isinstance(conflict_res_1, int)
    is_int_2 = isinstance(conflict_res_2, int)

    if is_int_1 and is_int_2:
      probablistic_conflict_resolution = False
    elif not is_int_1 and not is_int_2:
      probablistic_conflict_resolution = True
    else:
      raise ValueError("Conflict resolution specifications must be either both integers or both floats")

    return conflict_res_1, conflict_res_2, probablistic_conflict_resolution


  def find_src_conf_idx_from_desc(self, desc):
    idx = 0
    for src_conf in self._sinks_sources:
      if desc == src_conf["Description"]:
        return idx
      idx += 1
    raise ValueError("Description not found in source sink configs.")
  
  @staticmethod
  def conflict_resolution_key_from_src_conf_desc(desc1, desc2):
    return "{}/{}".format(desc1, desc2)


  def create_source_config_agents(self, agent_states, agent_geometries, 
                        behavior_models, execution_models, dynamic_models,
                        goal_definitions, controlled_agent_ids, world, agent_params, **kwargs):
    num_agents = len(agent_states)
    if any(len(lst) != num_agents for lst in [
      agent_geometries, behavior_models, execution_models, dynamic_models, goal_definitions, controlled_agent_ids]):
      raise ValueError("Config readers did not return equal sized of lists")
    agents = []
    for idx, agent_state in enumerate(agent_states):
      bark_agent = Agent( np.array(agent_state), 
                          behavior_models[idx], 
                          dynamic_models[idx],
                          execution_models[idx], 
                          agent_geometries[idx],
                          agent_params,
                          goal_definitions[idx],
                          world.map )
      if "agent_ids" in kwargs:
        bark_agent.SetAgentId(kwargs["agent_ids"][idx])
      else:
        bark_agent.SetAgentId(idx)     
      agents.append(bark_agent)
    return agents

  def eval_configuration(self, sink_source_config, config_type, args, kwargs):
    eval_config = sink_source_config[config_type]
    eval_config_type = eval_config["Type"]
    try:
      config_reader = eval("{}(self._random_state, self._current_scenario_idx)".format(eval_config_type))
    except NameError as error:
        for module in get_config_reader_modules():
          try:
            reader_type = getattr(module, eval_config_type)
            config_reader = eval("reader_type(self._random_state, self._current_scenario_idx)")
          except:
            pass
        if not config_reader:
          raise ValueError("Config reader type {} not found in added module paths".format(eval_config_type))
    config_return  = config_reader.create_from_config(eval_config, *args, **kwargs)
    self.add_config_reader_parameter_servers(sink_source_config["Description"], config_type, config_reader)
    return config_return

  @staticmethod
  def get_road_corridor_from_source_sink(source_sink_properties, map_interface):
    # generate road corridor between source and sink
    source_sink = source_sink_properties["SourceSink"]
    road_corridor = None
    if isinstance(source_sink, tuple) and \
           isinstance(source_sink[0], int):
           # road id given for start and end of road corridor 
           start_road_id = source_sink[0]
           end_road_id = source_sink[1]
           road_corridor = map_interface.GenerateRoadCorridor(start_road_id,
                                                end_road_id)
    elif isinstance(source_sink, list) and \
           isinstance(source_sink[0], list)  and \
           isinstance(source_sink[0][0], float):
           # point 2d given to find start and end road id road corridor
            goal_polygon = Polygon2d([0, 0, 0],
                                  [Point2d(-1,0),
                                  Point2d(-1,1),
                                  Point2d(1,1),
                                  Point2d(1,0)])
            start_point = Point2d(source_sink[0][0], source_sink[0][1])
            end_point = Point2d(source_sink[1][0], source_sink[1][1])
            goal_polygon = goal_polygon.Translate(end_point)
            road_corridor = map_interface.GenerateRoadCorridor(start_point,
                                                goal_polygon)
    else:
      raise ValueError("Unknown specification of start and end \
              of source sink config {}".format(source_sink))
    
    if not road_corridor:
      raise ValueError("No road corridor found betwen source {} \
              "" and sink {}".format(source_sink[0], source_sink[1]) )

    return road_corridor
