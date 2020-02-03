# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT


from modules.runtime.scenario.scenario_generation.config_readers.config_readers_interfaces import ConfigGoalDefinitions

from bark.world.goal_definition import GoalDefinition, GoalDefinitionPolygon, GoalDefinitionStateLimits
from modules.runtime.commons.parameters import ParameterServer


# helper class to support various types of goals easily generated for the 
# goal config readers
class GoalGenerator:
  @staticmethod
  def get_goal_definition(config_param_object, goal_definition, road_corridor, lane_position):
    return eval("GoalGenerator.{}(config_param_object, road_corridor, lane_position)".format(goal_definition))

  @staticmethod
  def EndOfLane(config_param_object, road_corridor, lane_position):
      lane_corridor = road_corridor.lane_corridors[lane_position]
      return fill_goal_def_state_lim_frenet(config_param_object, lane_corridor.center_line)

  @staticmethod
  def LaneChangeRight(config_param_object, road_corridor, lane_position):
    lane_position_right = lane_position+1
    if lane_position_right <  len(road_corridor.lane_corridors):
      lane_corridor_right = road_corridor.lane_corridors[lane_position_right]
      return fill_goal_def_state_lim_frenet(config_param_object, lane_corridor_right.center_line)
    else:
      raise ValueError("No lane change right possible with this \
               road corridor at lane position {}".format(lane_position))

  @staticmethod
  def LaneChangeLeft(config_param_object, road_corridor, lane_position):
    lane_position_left = lane_position-1
    if lane_position_left >= 0:
      lane_position_left = road_corridor.lane_corridors[lane_position_left]
      return fill_goal_def_state_lim_frenet(config_param_object, lane_position_left.center_line)
    else:
      raise ValueError("No lane change left possible with this \
               road corridor at lane position {}".format(lane_position))

  @staticmethod
  def fill_goal_def_state_lim_frenet(config_param_object, line_string):
    lateral_max_dist = config_param_object["MaxLateralDist", "Pair with values between 0,1: \
          Lateral maximum distance allowed to both sides of center line, normalized by lanewidth", (0.05, 0.05)]
    long_range = config_param_object["LongitudinalRange" , "Pair with values between 0,1: \
          Goal is within this longitudinal part of center line, normalized by lanewidth", (0.8, 1)]
    max_orientation_diff = config _param_object["MaxOrientationDifference" , "Pair with values between 0,pi: \
          Orientation must be within orientation limits around tangent angle of center line", (0.8, 1)]
    velocity_range = config _param_object["VelocityRange" , "Pair velocity values specifying allowed range", (0.8, 1)]

    goal_line_string = get_line_from_s_interval(line_string, long_range[0], long_range[1])
    goal_definition = GoalDefinitionStateLimitsFrenet(goal_line_string, lateral_max_dist, max_orientation_diff, velocity_range)
    return goal_definition


class FixedGoalTypes(ConfigGoalDefinitions):
  @abstractmethod
  def create_from_config(config_param_object, road_corridor, agent_states, controlled_agent_ids, **kwargs):
    self._controlled_agents_goal_type = config_param_object["GoalTypeControlled", "Specifies type of goals \
                          for controlled agents (EndOfLane, LaneChangeLeft, LaneChangeRight)", "EndOfLane"]
    
    self._other_agents_goal_type = config_param_object["GoalTypeOthers", "Specifies type of goals \
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


