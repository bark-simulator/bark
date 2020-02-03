# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT



from modules.runtime.scenario.scenario_generation.config_readers.config_readers_interfaces import ConfigReaderExecutionModels

from bark.models.execution import *
from modules.runtime.commons.parameters import ParameterServer

# this config reader execution behavior models with fixed type for all agents
class SingleFixedType(ConfigReaderExecutionModels):
  def create_from_config(config_param_object, road_corridor, agent_states,  **kwargs):

    execution_models = []
    for _ in agent_states:
      bark_model = self.model_from_config_params(config_param_object)
      execution_models.append(bark_model)
    return execution_models 

  def default_params(self):
    return {"ModelType": "ExecutionModelInterpolate"}

  def model_from_config_params(self, config_params):
    model_type = config_params.pop("ModelType", self.default_params["ModelType"])
    params = ParameterServer(config_params)
    bark_model = eval("{}(params)".format(model_type))    
    return bark_model
