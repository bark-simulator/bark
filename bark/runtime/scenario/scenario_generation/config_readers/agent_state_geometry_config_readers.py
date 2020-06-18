# Copyright (c) 2020 Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import numpy as np

from bark.runtime.scenario.scenario_generation.config_readers.config_readers_interfaces \
       import ConfigReaderAgentStatesAndGeometries

from bark.core.geometry.standard_shapes import *
from bark.core.geometry import *

# this config reader defines agent states with distances sampled uniformly standard vehicle geometries models
# it can be specified with parameter "lane_position" being between 1 and num_road_corridor_lanes 
# in what lanes vehicles are placed, or if None in all lanes they are placed
class UniformVehicleDistribution(ConfigReaderAgentStatesAndGeometries):
  def create_from_config(self, config_param_object, road_corridor):
    self._lane_positions = config_param_object["LanePositions", "List of values out of 0:max_num_lanes-1. \
            Vehicles are placed only this lanes \
           or None if vehicles shall be placed at alles lanes", None]
    self._vehicle_distance_range = config_param_object["VehicleDistanceRange",
      "Distance range between vehicles in meter given as tuple from which" + \
      "distances are sampled uniformly",
      (10, 20)]
    self._other_velocity_range = config_param_object["OtherVehicleVelocityRange",
      "Lower and upper bound of velocity in km/h given as tuple from which" + \
      " velocities are sampled uniformly",
      (20,30)]
    self._s_range = config_param_object["SRange", "pair of values (tuple) between 0 and 1 to limit placement  \
                     to certain part of road corridor", (0.1, 0.7)]
    # todo make parameterizable, but not only based on 2d points
    self._vehicle_2d_shape = CarLimousine()

    agent_states = []
    agent_geometries = []
    agent_lane_positions = []

    lane_corridors, lane_positions = self.select_lane_corridors(road_corridor, self._lane_positions)
    for idx, lane_corridor in enumerate(lane_corridors):
      tmp_agent_states, tmp_agent_geometries = self.agents_along_lane_corridor(lane_corridor,
                   self._s_range[0], self._s_range[1])

      agent_states.extend(tmp_agent_states)
      agent_geometries.extend(tmp_agent_geometries)
      agent_lane_positions.extend([lane_positions[idx]]*len(tmp_agent_states))

    assert(len(agent_states) == len(agent_geometries))
    assert(len(agent_states) == len(agent_lane_positions))
    return agent_states, agent_geometries, {"agent_lane_positions": agent_lane_positions}, config_param_object


  def select_lane_corridors(self, road_corridor, lane_positions):
    lane_corridors = []
    if lane_positions:
      for lane_position in lane_positions:
        lane_corridors.append(road_corridor.lane_corridors[lane_position])
    else:
        lane_corridors = road_corridor.lane_corridors
        lane_positions = list(range(1, len(lane_corridors)+1))
    return lane_corridors, lane_positions

  def sample_velocity_uniform(self, velocity_range):
    return self.random_state.uniform(velocity_range[0], velocity_range[1])

  def sample_distance_uniform(self, distance_range):
    return self.random_state.uniform(distance_range[0], distance_range[1])


  def agents_along_lane_corridor(self,
                              lane_corridor,
                              s_start,
                              s_end):
    linestring = lane_corridor.center_line
    agent_states = []
    agent_geometries = []
    corridor_length = lane_corridor.center_line.Length()
    s = s_start*corridor_length
    while s < s_end*corridor_length:
      # set agent state on linestring with random velocity
      xy_point =  GetPointAtS(linestring, s)
      angle = GetTangentAngleAtS(linestring, s)
      
      velocity = self.sample_velocity_uniform(self._other_velocity_range)
      agent_state = [0, xy_point.x(), xy_point.y(), angle, velocity ]

      agent_states.append(agent_state)
      agent_geometries.append(self._vehicle_2d_shape)

      # move forward on linestring based on vehicle size and max/min distance
      s += self._vehicle_2d_shape.front_dist + self._vehicle_2d_shape.rear_dist + \
                  self.sample_distance_uniform(self._vehicle_distance_range)
    return agent_states, agent_geometries

