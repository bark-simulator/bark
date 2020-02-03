# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT



from modules.runtime.scenario.scenario_generation.config_readers.config_readers_interfaces import ConfigReaderDynamicModels

from bark.models.dynamic import *
from modules.runtime.commons.parameters import ParameterServer

  # this config reader defines dynamic models with fixed type for all agents
class SingleFixedType(ConfigReaderDynamicModels):
  def create_from_config(config_param_object, road_corridor, agent_states,  **kwargs):
    model_type = config_param_object["ModelType", "Type of dynamic model", "SingleTrackModel"]
    dynamic_models = []
    for _ in agent_states:
      bark_model = self.model_from_model_type(model_type)
      behavior_models.append(bark_model)
    return dynamic_models, config_param_object

  def default_params(self):
    return {"ModelType": "SingleTrackModel"}

  def model_from_model_type(self, model_type):
    params = ParameterServer(config_params)
    bark_model = eval("{}(params)".format(model_type))    
    return bark_model
