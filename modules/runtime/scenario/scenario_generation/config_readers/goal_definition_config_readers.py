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
      #define a polygon exactly the width of the lane
      s_offset = config_param_object["SOffset"]["Goal Polygon s-offset (in 0,1) from end of lane", 0.1]
      s_offset = config_param_object["SLength"]["Goal Polygon length in s-coordinates (0,1)", 0.05]

      lane_corridor = road_corridor.lane_corridors[lane_position]
      cl = lane_corridor.center_line
      rl = lane_corridor.right_boundary
      ll = lane_corridor.left_boundary
      last_point_cl = cl.toArray()[-1]
      last_point_rl = rl.toArray()[-1]
      last_point_ll = ll.toArray()[-1]
      lane_width = sqrt
      width_lane_last_point = 
      goal_polygon = Polygon2d()
      goal_polygon = goal_polygon.translate(Point2d(self._others_sink[idx][0],
                                                    self._others_sink[idx][1]))
      goal_definition = GoalDefinitionPolygon(goal_polygon)
      return goal_definition

  @staticmethod
  def LaneChangeRight(config_param_object, road_corridor, lane_position):
    pass

  @staticmethod
  def LaneChangeLeft(config_param_object, road_corridor, lane_position):
    pass

  
  @staticmethod
  def GoalDefinitionStateLimitsNearLineString(config_param_object, line_string):
    lateral_range = config_param_object["LateralRange", "Pair with values between 0,1: \
          Lateral variations of state allowed to both sides of center line, normalized by lanewidth", [0.05, 0.05]]
    long_range = config_param_object["LongitudinalRange" , "Pair with values between 0,1: \
          Goal is within this longitudinal part of center line, normalized by lanewidth", [0.8, 1]]
    orientation_range = config _param_object["OrientationRange" , "Pair with values between 0,pi: \
          Orientation must be within orientation variation around center line", [0.8, 1]]
    goal_limits_left = goal_center_line.translate(Point2d(-lims[0], -lims[1]))
    goal_limits_right = goal_center_line.translate(Point2d(lims[0], lims[1]))
    goal_limits_right.reverse()
    goal_limits_left.append_linestring(goal_limits_right)
    polygon = Polygon2d([0,0,0], goal_limits_left)

    goal_definition = GoalDefinitionStateLimits(polygon, (1.57-0.08, 1.57+0.08))
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


