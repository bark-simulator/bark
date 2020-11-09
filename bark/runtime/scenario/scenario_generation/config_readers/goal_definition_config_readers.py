# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.


from bark.runtime.scenario.scenario_generation.config_readers.config_readers_interfaces import ConfigReaderGoalDefinitions

from bark.core.world.goal_definition import GoalDefinitionStateLimitsFrenet, GoalDefinitionSequential
from bark.runtime.commons.parameters import ParameterServer

from bark.core.geometry import *
import logging

# helper class to support various types of goals easily generated for the 
# goal config readers
class GoalGenerator:
  @staticmethod
  def get_goal_definition(config_param_object, goal_definition, road_corridor, lane_position, enforce_goal):
    if enforce_goal:
      return eval("GoalGenerator.{}(config_param_object, road_corridor, lane_position)".format(goal_definition))
    else:
      try:
        return eval("GoalGenerator.{}(config_param_object, road_corridor, lane_position)".format(goal_definition))
      except:
        return eval("GoalGenerator.EndOfLane(config_param_object, road_corridor, lane_position)")

  @staticmethod
  def EndOfLane(config_param_object, road_corridor, lane_position):
      lane_corridor = road_corridor.lane_corridors[lane_position]
      return GoalGenerator.fill_goal_def_state_lim_frenet(config_param_object, lane_corridor.center_line)

  @staticmethod
  def LaneChangeRight(config_param_object, road_corridor, lane_position):
    lane_position_right = lane_position+1
    if lane_position_right <  len(road_corridor.lane_corridors):
      lane_corridor_right = road_corridor.lane_corridors[lane_position_right]
      return GoalGenerator.fill_goal_def_state_lim_frenet(config_param_object, lane_corridor_right.center_line)
    else:
      raise ValueError("No lane change right possible with this \
               road corridor at lane position {}".format(lane_position))

  @staticmethod
  def LaneChangeLeft(config_param_object, road_corridor, lane_position):
    lane_position_left = lane_position-1
    if lane_position_left >= 0:
      lane_position_left = road_corridor.lane_corridors[lane_position_left]
      return GoalGenerator.fill_goal_def_state_lim_frenet(config_param_object, lane_position_left.center_line)
    else:
      raise ValueError("No lane change left possible with this \
               road corridor at lane position {}".format(lane_position))

  @staticmethod
  def fill_goal_def_state_lim_frenet(config_param_object, line_string):
    lateral_max_dist = config_param_object["MaxLateralDist", "Pair with values between 0,1: \
          Lateral maximum distance allowed to both sides of center line, normalized by lanewidth", (0.1, 0.1)]
    long_range = config_param_object["LongitudinalRange" , "Pair with values between 0,1: \
          Goal is within this longitudinal part of center line, normalized by lanewidth", (0, 1.0)]
    max_orientation_diff = config_param_object["MaxOrientationDifference" , "Pair with values between 0,pi: \
          Orientation must be within orientation limits around tangent angle of center line", (0.08, 0.08)]
    velocity_range = config_param_object["VelocityRange" , "Pair velocity values specifying allowed range", (10, 20)]

    length = line_string.Length()
    goal_line_string = GetLineFromSInterval(line_string, long_range[0]*length, long_range[1]*length)
    goal_definition = GoalDefinitionStateLimitsFrenet(goal_line_string, lateral_max_dist, max_orientation_diff, velocity_range)
    return goal_definition

  @staticmethod
  def make_sequential(goal_definition, num_steps):
    goal_sequence = [goal_definition]*num_steps
    goal_def_seq = GoalDefinitionSequential(goal_sequence)
    return goal_def_seq


class FixedGoalTypes(ConfigReaderGoalDefinitions):
  def create_from_config(self, config_param_object, road_corridor, agent_states, controlled_agent_ids, **kwargs):
    self._controlled_agents_goal_type = config_param_object["GoalTypeControlled", "Specifies type of goals \
                          for controlled agents (EndOfLane, LaneChangeLeft, LaneChangeRight)", "EndOfLane"]
    self._enforce_controlled_goal = config_param_object["EnforceControlledGoal", "If true exception is raised if goal not available", True]
    
    self._other_agents_goal_type = config_param_object["GoalTypeOthers", "Specifies type of goals \
                          for other agents (EndOfLane, LaneChangeLeft, LaneChangeRight)", "EndOfLane"]
    self._enforce_others_goal = config_param_object["EnforceOthersGoal", "If true exception is raised if goal not available", True]
    self._use_sequential_goal_controlled = config_param_object["SequentialGoalEgo", "If true sequential goals for ego are added", False]
    self._num_sequential_steps = config_param_object["SequentialGoalNumSteps", "Numbre of sequential goals when true", 4]

    goal_definitions = []
    agent_lane_positions = kwargs.pop("agent_lane_positions")
    for idx, _ in enumerate(agent_states):
      lane_position = agent_lane_positions[idx]
      if isinstance(lane_position, list):
        lane_position = lane_position[0]
      if controlled_agent_ids[idx]:
          goal_definition = GoalGenerator.get_goal_definition(
            config_param_object, self._controlled_agents_goal_type, road_corridor, lane_position, self._enforce_controlled_goal)
          if self._use_sequential_goal_controlled:
            goal_definition = GoalGenerator.make_sequential(goal_definition, self._num_sequential_steps)
      else:
          goal_definition = GoalGenerator.get_goal_definition(
          config_param_object, self._other_agents_goal_type, road_corridor, lane_position, self._enforce_others_goal)
      goal_definitions.append(goal_definition)
    return goal_definitions, {}, config_param_object




