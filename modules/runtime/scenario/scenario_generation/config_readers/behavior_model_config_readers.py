# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT



from modules.runtime.scenario.scenario_generation.config_readers.config_readers_interfaces import ConfigReaderBehaviorModels
from modules.runtime.scenario.scenario_generation.config_readers.model_conversion import ModelConversion

from bark.models.behavior import *
from modules.runtime.commons.parameters import ParameterServer


class SingleFixedType(ConfigReaderBehaviorModels):
  def create_from_config(config_param_object, road_corridor, agent_states,  **kwargs):

    behavior_models = []
    for _ in agent_states:
      bark_model = self.model_from_config_params(config_param_object)
      behavior_models.append(bark_model)
    return behavior_models, 

  def default_params(self):
    return {"ModelType": "BehaviorConstantVelocity"}

  def model_from_config_params(self, config_params):
    model_type = config_params.pop("ModelType", self.default_params["ModelType"])
    params = ParameterServer(config_params)
    bark_model = eval("{}(params)".format(model_type))    
    return bark_model
