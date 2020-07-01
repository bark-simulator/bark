# Copyright (c) 2020 Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.



from bark.runtime.scenario.scenario_generation.config_readers.config_readers_interfaces import ConfigReaderBehaviorModels

from bark.core.models.behavior import *
from bark.runtime.commons.parameters import ParameterServer

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
    model_default_params = ParameterServer(json={})
    for _ in agent_states:
      bark_model, params = self.model_from_model_type(model_type, model_params)
      behavior_models.append(bark_model)
      behavior_model_types.append(model_type)
    return behavior_models, {"behavior_model_types" : behavior_model_types}, config_param_object

  def model_from_model_type(self, model_type, params):
    bark_model = eval("{}(params)".format(model_type))    
    return bark_model, params

  def get_param_servers(self):
    return self.param_servers


class SampleBehaviorType(ConfigReaderBehaviorModels):
  def __init__(self, random_state):
    super().__init__(random_state)
    self.param_servers = []

  def create_from_config(self, config_param_object, road_corridor, agent_states,  **kwargs):
    model_types = config_param_object["ModelTypesList", "Type of behavior model" \
                "used for all vehicles", ["BehaviorIDMClassic", "BehaviorMobil"]]
    model_params = config_param_object.AddChild("ModelParams")
    # ----- DEFAULT PARAMETER HANDLING
    # based on types retrieve default params which are maintained as scenario defaults
    for model_type in model_types:
        behavior_params = model_params.AddChild(model_type)
        _, _ = self.model_from_model_type(model_type, behavior_params)
        #param server must be persisted for each behavior to enable serialization of parameters
        

    #------ BEHAVIOR MODEL SAMPLING
    behavior_models = []
    behavior_model_types = []
    for _ in agent_states:
        model_idx = self.random_state.randint(low=0, high=len(model_types), size=None) 
        model_type = model_types[model_idx]
        model_type_params = model_params.AddChild(model_type)
        params = ParameterServer()
        bark_model, params = self.model_from_model_type(model_type, model_type_params)
        self.param_servers.append(model_type_params) 
        behavior_models.append(bark_model)
        behavior_model_types.append(model_type)
    return behavior_models, {"behavior_model_types" : behavior_model_types}, config_param_object

  def model_from_model_type(self, model_type, params):
    bark_model = eval("{}(params)".format(model_type))    
    return bark_model, params

  def get_param_servers(self):
    return self.param_servers