

import abc

# The property-based scenario generation must implement all of this interfaces. Property readers only the respective interface
class PropertyReaderBehaviorModels:
  # returns list of size num agents with behavior models based on property, default_params_dict
  @abstractmethod
  def create_from_properties(property_param_object, road_corridor, agent_states,  **kwargs):
    pass

class PropertyReaderExecutionModels:
  # returns list of size num agents with execution models based on property, default_params_dict
  @abstractmethod
  def create_from_properties(property_param_object, road_corridor, agent_states,  **kwargs):
    pass

class PropertyReaderDynamicModels:
  # returns list of size num agents with dynamic models based on property, default_params_dict
  @abstractmethod
  def create_from_properties(property_param_object, road_corridor, agent_states,  **kwargs):
    pass

class PropertyReaderAgentStatesAndGeometries:
  @abstractmethod
  # returns first a tuple containing 1] list of size num agents each element being a python list representing an agent state
  # and 2] a list of size num agents of point lists defining the agent geometries
  #  3] None or a dict passed as kwargs to the other functions (
  # e.g. the dict can contain a type entry specifying for each agent if it is a truck or car)
  # and second a default_params_dict
  def create_from_properties(property_param_object, road_corridor):
    pass

class PropertyReaderDynamicModels:
  # returns list of size num agents with behavior models based on property, default_params_dict
  @abstractmethod
  def create_from_properties(property_param_object, road_corridor, agent_states,  **kwargs):
    pass


class PropertyReaderGoalDefinitions:
  # returns list of size num agents with goal definitions for each agent based on property, default_params_dict
  @abstractmethod
  def create_from_properties(property_param_object, road_corridor, agent_states,  **kwargs):
    pass

class PropertyReaderGoalDefinitions:
   # returns list of size num agents with true or false depending if agent is controlled or not for each agent based on property, default_params_dict
  @abstractmethod
  def create_from_properties(property_param_object, road_corridor, agent_states,  **kwargs):
    pass
