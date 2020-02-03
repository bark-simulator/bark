# Copyright (c) 2020 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT



from modules.runtime.scenario.scenario_generation.config_readers.config_readers_interfaces import ConfigReaderBehaviorModels

from bark.models.behavior import *
from modules.runtime.commons.parameters import ParameterServer

  # this config reader defines behavior models with fixed type for all agents
class SingleFixedType(ConfigReaderBehaviorModels):
  def create_from_config(config_param_object, road_corridor, agent_states,  **kwargs):
    model_type = config_param_object["ModelType", "Type of behavior model \
                used for all vehicles", "BehaviorIDMClassic"]
    behavior_models = []
    behavior_model_types = []
    for _ in agent_states:
      bark_model = self.model_from_model_type(model_type)
      behavior_models.append(bark_model)
      behavior_model_types.append(model_type)
    return behavior_models, {"behavior_model_types" : behavior_model_types}, config_param_object

  def model_from_model_type(self, model_type):
    params = ParameterServer(config_params)
    bark_model = eval("{}(params)".format(model_type))    
    return bark_model
