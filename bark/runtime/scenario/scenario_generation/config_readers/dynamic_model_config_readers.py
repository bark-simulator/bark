# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.



from bark.runtime.scenario.scenario_generation.config_readers.config_readers_interfaces import ConfigReaderDynamicModels

from bark.core.models.dynamic import *
from bark.runtime.commons.parameters import ParameterServer

  # this config reader defines dynamic models with fixed type for all agents
class FixedDynamicType(ConfigReaderDynamicModels):
  def create_from_config(self, config_param_object, road_corridor, agent_states,  **kwargs):
    model_type = config_param_object["ModelType", "Type of dynamic model \
                used for all vehicles", "SingleTrackModel"]
    model_params = config_param_object.AddChild("ModelParams")
    dynamic_models = []
    for _ in agent_states:
      bark_model = self.model_from_model_type(model_type, model_params)
      dynamic_models.append(bark_model)
    return dynamic_models, {}, config_param_object

  def default_params(self):
    return {"ModelType": "SingleTrackModel"}

  def model_from_model_type(self, model_type, params):
    bark_model = eval("{}(params)".format(model_type))    
    return bark_model
