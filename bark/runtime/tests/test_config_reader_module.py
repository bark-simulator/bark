# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

from bark.runtime.scenario.scenario_generation.config_readers.config_readers_interfaces import ConfigReaderBehaviorModels

from bark.core.models.behavior import *
from bark.runtime.commons.parameters import ParameterServer

class TestReaderFixedBehaviorType(ConfigReaderBehaviorModels):
  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
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