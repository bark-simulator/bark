# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT



from modules.runtime.scenario.scenario_generation.config_readers.config_readers_interfaces import ConfigReaderExecutionModels

from bark.models.execution import *
from modules.runtime.commons.parameters import ParameterServer

# this config reader execution behavior models with fixed type for all agents
class FixedExecutionType(ConfigReaderExecutionModels):
  def create_from_config(self, config_param_object, road_corridor, agent_states,  **kwargs):
    model_type = config_param_object["ModelType", "Type of dynamic model \
                used for all vehicles", "ExecutionModelInterpolate"]
    execution_models = []
    for _ in agent_states:
      bark_model = self.model_from_model_type(model_type)
      execution_models.append(bark_model)
    return execution_models, {},  config_param_object

  def default_params(self):
    return {"ModelType": "ExecutionModelInterpolate"}

  def model_from_model_type(self, model_type):
    params = ParameterServer()
    bark_model = eval("{}(params)".format(model_type))    
    return bark_model
