# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import numpy as np
import glob
from collections import defaultdict

from bark.runtime.scenario.scenario_generation.config_readers.config_readers_interfaces \
       import ConfigReaderAgentStatesAndGeometries

from bark.runtime.scenario.interaction_dataset_processing import ShapeFromTrack, \
    BarkStateFromMotionState, InitStateFromTrack
from com_github_interaction_dataset_interaction_dataset.python.utils import dataset_reader

from bark.core.geometry.standard_shapes import *
from bark.core.geometry import *
from bark.core.models.dynamic import *

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

class InteractionDataTrackIdsStatesGeometries(ConfigReaderAgentStatesAndGeometries):
  def create_from_config(self, config_param_object, road_corridor):
    track_file_name = config_param_object["TrackFilename", "Path to track file (csv)",
                                        "bark/runtime/tests/data/interaction_dataset_DEU_Merging_dummy_track.csv"]
    track_ids = config_param_object["TrackIds", "IDs of the vehicle tracks to import.", [1]]
    start_time = config_param_object["StartTs", "Timestamp when to start the scenario (ms)", 0]
    end_time = config_param_object["EndTs","Timestamp when to end the scenario (ms)", None]
    xy_offset = config_param_object["XYOffset", "offset in x and y direction.", [0, 0]]

    agent_geometries = []
    agent_states = []
    lane_positions = []
    tracks = []
    for track_id in track_ids:
      tracks_read = dataset_reader.read_tracks(track_file_name)
      track = tracks_read[track_id]
      if start_time is None:
          start_time = track.time_stamp_ms_first
      if end_time is None:
          end_time = track.time_stamp_ms_last
      numpy_state = InitStateFromTrack(track, xy_offset, start_time)
      agent_state = numpy_state.reshape(5).tolist()
      agent_states.append(agent_state)

      shape = ShapeFromTrack(track)
      agent_geometries.append(shape)

      tracks.append(track)

      lane_positions_agent = self.find_lane_positions(numpy_state, road_corridor)
      lane_positions.append(lane_positions_agent)

    assert(len(agent_states) == len(agent_geometries))
    return agent_states, agent_geometries, {"track_ids": track_ids, "tracks" : tracks, \
             "agent_ids" : track_ids, "xy_offset" : xy_offset, "start_time" : start_time, "end_time" : end_time, \
               "agent_lane_positions" : lane_positions}, config_param_object
  
  def find_lane_positions(self, init_state, road_corridor):
    lps = []
    for idx, lane_corridor in enumerate(road_corridor.lane_corridors):
      if Collide(lane_corridor.polygon, Point2d(init_state[int(StateDefinition.X_POSITION)], \
         init_state[int(StateDefinition.Y_POSITION)])):
        lps.append(idx)
    return lps

class InteractionDataWindowStatesGeometries(ConfigReaderAgentStatesAndGeometries):
  window_start = None
  window_end = None
  track_dict_list = None
  current_track_file = None
  def create_from_config(self, config_param_object, road_corridor):
    track_file_names = config_param_object["TrackFilenames", "Path to track file (csv)",
                                        "bark/runtime/tests/data/*_dataset_dummy_track.csv"]
    xy_offset = config_param_object["XYOffset", "offset in x and y direction.", [0, 0]]
    wheel_base = config_param_object["WheelBase", "Wheelbase assumed for shape calculation", 2.7]
    window_length = config_param_object["WindowLength", "Window length for search of agents for a scenario ", 200]
    skip_time_scenarios = config_param_object["SkipTimeScenarios", "Time delta between start of previous scenario window and next init of search window", 0]
    skip_time_search = config_param_object["SkipTimeSearch", "Time delta between start of current and next search window", 100]
    min_time = config_param_object["MinTime", "Time offset from beginning of track file to start searching", 0]
    max_time = config_param_object["MaxTime", "Max time included in search", 100000]
    only_on_one_lane = config_param_object["OnlyOnOneLane", "If True only scenarios are defined where agents are on a single lane", False]
    minimum_numbers_per_lane = config_param_object["MinimumNumbersPerLane", "List where each element specifies how man vehicles must be at minimum at this lane,\
                                  lane position equals list index", [1, 0]]

    window_start = InteractionDataWindowStatesGeometries.window_start 
    window_end = InteractionDataWindowStatesGeometries.window_end
    track_dict_list = InteractionDataWindowStatesGeometries.track_dict_list
    current_track_file = InteractionDataWindowStatesGeometries.current_track_file
    # reset when a new scenario generation starts
    if self.current_scenario_idx == 0:
      window_start = min_time
      window_end = min_time + window_length
      track_dict_list = self.load_tracks(track_file_names)
      current_track_file = 0
    else:
      window_start += skip_time_scenarios
      window_end += skip_time_scenarios

    scenario_track_ids = []
    while len(scenario_track_ids) == 0 and current_track_file < len(track_dict_list):
      track_dict = track_dict_list[current_track_file]
      scenario_track_ids, window_start, window_end = self.find_track_ids_moving_window(xy_offset, window_start, window_end, track_dict, only_on_one_lane, minimum_numbers_per_lane,
                        window_length, skip_time_search, min_time, max_time, road_corridor, wheel_base)
      if len(scenario_track_ids) > 0:
        break
      window_start = min_time
      window_end = min_time + window_length
      current_track_file += 1
    if len(scenario_track_ids) < 1:
      raise ValueError("No track ids found for scenario idx {}. Consider lowering the number of scenarios.".format(self.current_scenario_idx))

    agent_geometries = []
    agent_states = []
    lane_positions = []
    tracks = []
    for track_id in scenario_track_ids:
      track = track_dict[track_id]
      numpy_state = self.get_init_state(track, xy_offset, window_start)
      agent_state = numpy_state.reshape(5).tolist()
      agent_states.append(agent_state)
      shape = ShapeFromTrack(track)
      agent_geometries.append(shape)
      tracks.append(track)
      lane_positions_single = self.find_lane_positions(road_corridor, self.get_shape_at_time_point(track, xy_offset, window_start))
      lane_positions.append(lane_positions_single)

    assert(len(agent_states) == len(agent_geometries))
    InteractionDataWindowStatesGeometries.window_start = window_start
    InteractionDataWindowStatesGeometries.window_end = window_end
    InteractionDataWindowStatesGeometries.track_dict_list = track_dict_list
    InteractionDataWindowStatesGeometries.current_track_file = current_track_file

    return agent_states, agent_geometries, {"track_ids": scenario_track_ids, "tracks" : tracks, \
             "agent_ids" : scenario_track_ids, "xy_offset" : xy_offset, "start_time" : window_start, "end_time" : window_end, \
               "agent_lane_positions" : lane_positions}, config_param_object

  def load_tracks(self, track_filenames):
    track_dict_list = []
    if isinstance(track_filenames, list):
      track_files_found = track_filenames
    else:
      track_files_found = glob.glob(track_filenames)
    for filename in track_files_found:
      track_dict = dataset_reader.read_tracks(filename)
      track_dict_list.append(track_dict)
    return track_dict_list

  def find_track_ids_moving_window(self, xy_offset, window_start, window_end, track_dict, only_on_one_lane, minimum_numbers_per_lane, \
                                      window_length, skip_time_search, time_offset, max_time, road_corridor, wheel_base):
    def move_window(window_start, window_end):
      window_start += skip_time_search
      window_end = window_start + window_length
      return window_start, window_end

    while True:
      window_start, window_end = move_window(window_start, window_end)
      if window_end > max_time:
        return [], window_start, window_end
      window_track_ids = self.find_track_ids(track_dict, window_start, window_end)
      if len(window_track_ids) < 1:
        continue

      numbers_per_lane = defaultdict(list)
      for track_id in window_track_ids:
        track = track_dict[track_id]
        shape = self.get_shape_at_time_point(track, xy_offset, window_start)
        lane_positions = self.find_lane_positions(road_corridor, shape)
        # skip whole window if lane positions not fulfilled
        if only_on_one_lane and len(lane_positions) != 1:
          continue
        if len(lane_positions) == 0:
          continue
        if not Within(shape, road_corridor.polygon):
          continue
        numbers_per_lane[lane_positions[0]].append(track_id)

      valid_track_ids = []
      desired_number_valid = True
      for lane_pos, minimum_number in enumerate(minimum_numbers_per_lane):
        if len(numbers_per_lane[lane_pos]) < minimum_number:
          desired_number_valid = False
          break
        else:
          valid_track_ids.extend(numbers_per_lane[lane_pos])
      if desired_number_valid:
        break
      else:
        continue

    return valid_track_ids, window_start, window_end

  def get_init_state(self, track, xy_offset, start_time):
    return InitStateFromTrack(track, xy_offset, start_time)

  def find_track_ids(self, track_dict, start_time, end_time):
    list_ids = []
    for id_current in track_dict.keys():
        if track_dict[id_current].time_stamp_ms_last >= end_time and \
          track_dict[id_current].time_stamp_ms_first <= start_time:
            list_ids.append(id_current)
    return list_ids

  def get_shape_at_time_point(self, track, xy_offset, time_point):
    init_state = self.get_init_state(track, xy_offset, time_point)
    shape = ShapeFromTrack(track)
    shape_at_timepoint = shape.Transform([init_state[int(StateDefinition.X_POSITION)], \
                                   init_state[int(StateDefinition.Y_POSITION)], \
                                   init_state[int(StateDefinition.THETA_POSITION)]])
    return shape_at_timepoint

  def find_lane_positions(self, road_corridor, shape):
    lps = []
    for idx, lane_corridor in enumerate(road_corridor.lane_corridors):
      if Collide(lane_corridor.polygon, shape):
        lps.append(idx)
    return lps