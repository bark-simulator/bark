# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT


from modules.runtime.scenario.scenario_generation.config_readers.config_readers_interfaces import ConfigGoalDefinitions

from bark.models.behavior import *
from modules.runtime.commons.parameters import ParameterServer

# helper class to support various types of goals easily generated for the 
# goal config readers
class GoalGenerator:
  
  @staticmethod
  def get_goal_definition(config_param_object, goal_definition, road_corridor, lane_position):
    return eval("GoalGenerator.{}(config_param_object, road_corridor, lane_position)".format(goal_definition))

  @staticmethod
  def EndOfLane(config_param_object, road_corridor, lane_position):
    pass

  @staticmethod
  def LaneChangeRight(config_param_object, road_corridor, lane_position):
    pass

  @staticmethod
  def LaneChangeLeft(config_param_object, road_corridor, lane_position):
    pass


class FixedGoalTypes(ConfigGoalDefinitions):
  @abstractmethod
  def create_from_config(config_param_object, road_corridor, agent_states, controlled_agent_ids, **kwargs):
    self._controlled_agents_goal_type = config_param_object["GoalTypeControlled", "Specifies type of goals \
                          for controlled agents (EndOfLane, LaneChangeLeft, LaneChangeRight)", "EndOfLane"]
    
    self._other_agents_goal_type = config_param_object["GoalTypeControlled", "Specifies type of goals \
                          for other agents (EndOfLane, LaneChangeLeft, LaneChangeRight)", "EndOfLane"]
    
    goal_definitions = []
    for idx, _ in enumerate(agent_states):
      if controlled_agent_ids[idx]:
        goal_definition = GoalGenerator.get_goal_definition(
          config_param_object, self._controlled_agents_goal_type, road_corridor, **kwargs)
        )
      else:
          goal_definition = GoalGenerator.get_goal_definition(
          config_param_object, self._other_agents_goal_type, road_corridor, **kwargs)
        )
      goal_definitions.append(goal_definition)
    
    return goal_definitions, config_param_object


