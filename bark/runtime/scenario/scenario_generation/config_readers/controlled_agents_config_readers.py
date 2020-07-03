# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.


import numpy as np
import math

from bark.runtime.scenario.scenario_generation.config_readers.config_readers_interfaces import ConfigReaderControlledAgents

from bark.core.world.goal_definition import GoalDefinition, GoalDefinitionPolygon, GoalDefinitionStateLimits
from bark.runtime.commons.parameters import ParameterServer

# no one (in this road corridor) is a controlled agent
class NoneControlled(ConfigReaderControlledAgents):
   # returns list of size num agents with true or false depending if agent is controlled or not for each agent based on property, default_params_dict
  def create_from_config(self, config_param_object, road_corridor, agent_states,  **kwargs):
    return [False] * len(agent_states), {}, config_param_object

# Select one agent randomly as controlled agent
class RandomSingleAgent(ConfigReaderControlledAgents):
  # returns list of size num agents with true or false depending if agent is controlled or not for each agent based on property, default_params_dict
  def create_from_config(self, config_param_object, road_corridor, agent_states,  **kwargs):
    controlled_agent_idx = self.random_state.randint(low=0, high=len(agent_states), size=None) 
    controlled_list = [False] * len(agent_states)
    controlled_list[controlled_agent_idx] = True
    return controlled_list, {}, config_param_object


# Select one agent randomly as controlled agent
class PositioningSingleAgent(ConfigReaderControlledAgents):
  # returns list of size num agents with true or false depending if agent is controlled or not for each agent based on property, default_params_dict
  def create_from_config(self, config_param_object, road_corridor, agent_states,  **kwargs):
    lane_pos = config_param_object["LanePosition", "Which lane should agent be at", 0]
    where_on_lane = config_param_object["WhereOnLane", "Where on lane with placeholder num_agents", "math.floor(num_agents/2)"]
    lane_positions = kwargs["agent_lane_positions"]
    lane_pos_agent_idx = [idx for idx, e in enumerate(lane_positions) if e == lane_pos]
    controlled_list_idx = eval(where_on_lane.replace("num_agents", "{}".format(len(lane_pos_agent_idx))))
    controlled_list = [False] * len(agent_states)
    controlled_list[controlled_list_idx] = True
    return controlled_list, {}, config_param_object

class AgentIds(ConfigReaderControlledAgents):
  # returns list of size num agents with true or false depending if agent is controlled or not for each agent based on property, default_params_dict
  def create_from_config(self, config_param_object, road_corridor, agent_states,  **kwargs):
    controlled_ids = config_param_object["ControlledIds", "Agent ids which should be controlled", [0, 2]]
    agent_ids = kwargs["agent_ids"]
    controlled_list = []
    for id in agent_ids:
      if id in controlled_ids: 
        controlled_list.append(True)
      else:
        controlled_list.append(False)
    return controlled_list, {}, config_param_object
