

import abc

# The property-based scenario generation must implement all of this interfaces. Property readers only the respective interface
class PropertyReaderInterfaces(metaclass=abc.ABCMeta):
  # returns list of size num agents with behavior models based on property 
  @abc.abstractmethod
  def create_from_properties_behavior_models(property_param_object, road_corridor, agent_states,  **kwargs):
    pass

  # returns list of size num agents with execution models based on property 
  @abc.abstractmethod
  def create_from_properties_execution_models(property_param_object, road_corridor, agent_states, **kwargs):
    pass

  # returns list of size num agents with dynamic models based on property 
  @abc.abstractmethod
  def create_from_properties_dynamic_models(property_param_object, road_corridor, agent_states, **kwargs):
    pass

  # returns a tuple 1] list of size num agents each element being a python list representing an agent state
  # and 2] a list of size num agents of point lists defining the agent geometries
  #  3] None or a dict passed as kwargs to the other functions (
  # e.g. the dict can contain a type entry specifying for each agent if it is a truck or car)
  @abc.abstractmethod
  def create_from_properties_agent_state_and_geometry(property_param_object, road_corridor):
    pass

  # returns list of size num agents with goal definitions for each agent based on property 
  @abc.abstractmethod
  def create_from_properties_goal_definitions(property_param_object, road_corridor, agent_states, **kwargs):
    pass

  # returns list of size num agents with goal definitions for each agent based on property 
  @abc.abstractmethod
  def create_from_properties_control_agent_ids(property_param_object, road_corridor, num_agents, **kwargs):
    pass
