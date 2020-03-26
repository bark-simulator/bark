# Copyright (c) 2020 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import numpy as np

from modules.runtime.scenario.scenario_generation.config_readers.config_readers_interfaces import ConfigReaderBehaviorModels

from bark.models.behavior import *
from modules.runtime.commons.parameters import ParameterServer

  # this config reader defines behavior models with fixed type for all agents
class FixedBehaviorType(ConfigReaderBehaviorModels):
  def __init__(self, random_state):
    super().__init__(random_state)
    self.param_servers = []

  def create_from_config(self, config_param_object, road_corridor, agent_states,  **kwargs):
    model_type = config_param_object["ModelType", "Type of behavior model \
                used for all vehicles", "BehaviorIDMClassic"]
    model_params = config_param_object.AddChild("ModelParams")
    self.param_servers.append(model_params) # use the same param server for all models
    behavior_models = []
    behavior_model_types = []
    for _ in agent_states:
      bark_model, _ = self.model_from_model_type(model_type, model_params)
      behavior_models.append(bark_model)
      behavior_model_types.append(model_type)
    return behavior_models, {"behavior_model_types" : behavior_model_types}, config_param_object

  def model_from_model_type(self, model_type, params):
    bark_model = eval("{}(params)".format(model_type))
    return bark_model, params

  def get_param_servers(self):
    return self.param_servers


class ParameterSampling(ConfigReaderBehaviorModels):
  def __init__(self, random_state):
    super().__init__(random_state)
    self.param_servers = []

  def create_from_config(self, config_param_object, road_corridor, agent_states,  **kwargs):
    model_type = config_param_object["ModelType", "Type of behavior model \
                used for all vehicles", "BehaviorIDMStochasticHeadway"]

    # only to get the default params of default model type
    model_params = config_param_object.AddChild("ModelParams")
    bark_model, _ = self.model_from_model_type(model_type, model_params)

    # now do the true sampling
    behavior_models = []
    behavior_model_types = []
    
    for _ in agent_states:
      model_params_sampled = self._sample_params_from_param_ranges(model_params)
      self.param_servers.append(model_params_sampled)
      bark_model, _ = self.model_from_model_type(model_type, model_params_sampled)
      behavior_models.append(bark_model)
      behavior_model_types.append(model_type)
    return behavior_models, {"behavior_model_types" : behavior_model_types}, config_param_object

  def _sample_params_from_param_ranges(self, param_range_descriptions):
    """
    searches through param server to find distribution keys
    """
    param_dict = ParameterServer()
    for key, value in param_range_descriptions.store.items():
      if "Distribution" in key:
        if "Uniform" in key:
          param_dict[key] = self._sample_uniform_dist_params(value)
        elif "Gauss" in key:
          param_dict[key] = self._sample_gauss_dist_params(value)
      elif isinstance(value, ParameterServer):
        param_dict[key] = self._sample_params_from_param_ranges(value)
      else:
        param_dict[key] = value
    return param_dict

  def _sample_uniform_dist_params(self, params_distribution):
    max_range = params_distribution["MaxRange", "From what range are distribution bound sampled,", [0, 1]]
    max_width = params_distribution["MaxWidth", "What maximum width can distribution have", [0, 1]]

    lower_bound = self.random_state.uniform(max_range[0], max_range[1])
    upper_bound = self.random_state.uniform(lower_bound,  lower_bound + max_width)

    _ = params_distribution["LowerBound", "", lower_bound]
    _ = params_distribution["UpperBound", "", upper_bound]
    return params_distribution


  def _sample_gauss_dist_params(self, params_distribution):
    return {}

  def model_from_model_type(self, model_type, params):
    bark_model = eval("{}(params)".format(model_type))
    return bark_model, params

  def get_param_servers(self):
    return self.param_servers


