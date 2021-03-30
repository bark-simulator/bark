# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

from abc import abstractmethod

class ConfigReader:
  def __init__(self, random_state, current_scenario_idx):
  # holds a global random state which should be used for sampling in config readers
    self.__random_state = random_state
  # holds current scenario idx if config reader adjust settings according to scenario idx ( e.g. databased models)
    self.__current_scenario_idx = current_scenario_idx

  @property
  def random_state(self):
    return self.__random_state

  @property
  def current_scenario_idx(self):
    return self.__current_scenario_idx


# The property-based scenario generation must implement all of this interfaces. Property readers only the respective interface
class ConfigReaderBehaviorModels(ConfigReader):
  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
  # returns list of size num agents with behavior models based on property, default_params_dict
  @abstractmethod
  def create_from_config(self, config_param_object, road_corridor, agent_states,  **kwargs):
    pass 
  
  # reimplement as param servers behavior models are serialized
  def get_param_servers(self):
    return []

class ConfigReaderExecutionModels(ConfigReader):
  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
  # returns list of size num agents with execution models based on property, default_params_dict
  @abstractmethod
  def create_from_config(self, config_param_object, road_corridor, agent_states,  **kwargs):
    pass

  # only reimplement if param server for this object shall be serialized (currently not needed)
  def get_param_servers(self):
    return []

class ConfigReaderDynamicModels(ConfigReader):
  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
  # returns list of size num agents with dynamic models based on property, default_params_dict
  @abstractmethod
  def create_from_config(self, config_param_object, road_corridor, agent_states,  **kwargs):
    pass

  # only reimplement if param server for this object shall be serialized (currently not needed)
  def get_param_servers(self):
    return []

class ConfigReaderAgentStatesAndGeometries(ConfigReader):
  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
  @abstractmethod
  # returns a tuple containing 1] list of size num agents each element being a python list representing an agent state
  # and 2] a list of size num agents of point lists defining the agent geometries
  #  3] None or a dict passed as kwargs to the other functions (
  # e.g. the dict can contain a type entry specifying for each agent if it is a truck or car)
  # and 4] a default_params_dict
  def create_from_config(self, config_param_object, road_corridor):
    pass

  # only reimplement if param server for this object shall be serialized (currently not needed)
  def get_param_servers(self):
    return []

class ConfigReaderDynamicModels(ConfigReader):
  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
  # returns list of size num agents with behavior models based on property, default_params_dict
  @abstractmethod
  def create_from_config(self, config_param_object, road_corridor, agent_states,  **kwargs):
    pass

  # only reimplement if param server for this object shall be serialized (currently not needed)
  def get_param_servers(self):
    return []


class ConfigReaderGoalDefinitions(ConfigReader):
  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
  # returns list of size num agents with goal definitions for each agent based on property, default_params_dict
  @abstractmethod
  def create_from_config(self, config_param_object, road_corridor, agent_states, controlled_agent_ids,  **kwargs):
    pass

  # only reimplement if param server for this object shall be serialized (currently not needed)
  def get_param_servers(self):
    return []

class ConfigReaderControlledAgents(ConfigReader):
  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
   # returns list of size num agents with true or false depending if agent is controlled or not for each agent based on property, default_params_dict
  @abstractmethod
  def create_from_config(self, config_param_object, road_corridor, agent_states,  **kwargs):
    pass

  # only reimplement if param server for this object shall be serialized (currently not needed)
  def get_param_servers(self):
    return []


class ConfigReaderObserverModel(ConfigReader):
  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
  
  # returns list of size num agents with true or false depending if agent is controlled or not for each agent based on property, default_params_dict
  @abstractmethod
  def create_from_config(self, config_param_object,  **kwargs):
    pass

  # only reimplement if param server for this object shall be serialized (currently not needed)
  def get_param_servers(self):
    return []
