# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import os
import shutil
import logging

from bark.runtime.commons.parameters import ParameterServer
from bark.core.commons import FrenetState
from bark.runtime.scenario.interaction_dataset_processing.interaction_dataset_reader import *
from bark.runtime.scenario.interaction_dataset_processing.agent_track_info import AgentTrackInfo
from bark.runtime.scenario.interaction_dataset_processing.scenario_track_info import ScenarioTrackInfo
from bark.runtime.scenario.scenario import Scenario
from bark.core.geometry import Point2d, Collide, Within
from bark.core.geometry.standard_shapes import *

from com_github_interaction_dataset_interaction_dataset.python.utils import dataset_reader


class DatasetDecomposer:
    def __init__(self, map_interface, road_corridor, track_filename, vehicle_length_max, xy_offset, starting_offset_ms=0):
        self._map_interface = map_interface
        self._track_filename = track_filename
        self._starting_offset_ms = starting_offset_ms
        self._xy_offset = xy_offset
        self._vehicle_length_max = vehicle_length_max
        self._road_corridor = road_corridor
        self._track_dict = dataset_reader.read_tracks(track_filename)
        self._agents_track_infos = self.__setup_agents_track_infos__()

    def __find_first_ts_on_map__(self, id_ego):
        traj = TrajectoryFromTrack(
            self._track_dict[id_ego], xy_offset=self._xy_offset)
        for state in traj:
            # wb = WheelbaseFromTrack(self._track_dict[id_ego])
            # r = ColRadiusFromTrack(self._track_dict[id_ego])
            # agent_shape = GenerateCarLimousine(wb, r)
            agent_shape = CarLimousine()
            agent_shape = agent_shape.Transform([state[1], state[2], state[3]])
            for lc in self._road_corridor.lane_corridors:
                frenet_state = FrenetState(state, lc.center_line)
                inside_road = Within(agent_shape, self._road_corridor.polygon)
                aligned_with_road = abs(frenet_state.angle) < 0.1
                if inside_road and aligned_with_road:
                    ms_to_s = 1e3 # scale from s (BARK) to ms (dataset)
                    time_ego_first = state[0] * ms_to_s + self._starting_offset_ms
                    return time_ego_first

        return None

    def __setup_agents_track_infos__(self):
        # dictionary mapping first valid timestamp to agnet id
        agents_track_infos = {}
        for agent_id in self._track_dict.keys():
            # TODO: this could be made optional
            first_ts_on_map = self.__find_first_ts_on_map__(agent_id)
            track = self._track_dict[agent_id]
            if first_ts_on_map is None:
                logging.info("Skip, as agent {} not found on map".format(agent_id))
                pass
            elif str(track.agent_type) != "car":
                logging.info("Skip, as agent {} is of type {}".format(agent_id, track.agent_type))
                pass
            elif track.length > self._vehicle_length_max:
                logging.info("Skip, as agent {} exceeds max length of {} with length {}".format(agent_id, self._vehicle_length_max, track.length))
                pass
            else:
                start_time = first_ts_on_map
                end_time = track.time_stamp_ms_last
                new_agent = AgentTrackInfo(filename=self._track_filename, track_id=agent_id,
                                           start_time=start_time, end_time=end_time)
                agents_track_infos[agent_id] = new_agent
        return agents_track_infos

    def __get_agent_track_info__(self, agent_id):
        if agent_id in self._agents_track_infos.keys():
            return self._agents_track_infos[agent_id]
        else:
            raise ValueError("agent {} not available".format(agent_id))

    def __find_other_ids__(self, id_ego):
        # for each agent extract ids of other agents present in the same time span
        list_ids = []
        time_ego_first = self.__get_agent_track_info__(id_ego).GetStartTimeMs()
        time_ego_last = self.__get_agent_track_info__(id_ego).GetEndTimeMs()

        for id_o in self._agents_track_infos.keys():
            if id_ego == id_o:
                pass
            elif self.__get_agent_track_info__(id_o).GetEndTimeMs() <= time_ego_first:
                # other ends too early
                pass
            elif self.__get_agent_track_info__(id_o).GetStartTimeMs() >= time_ego_last:
                # other starts too late
                pass
            else:
                list_ids.append(id_o)

        return list_ids

    def __collect_all_scenario_infos__(self):
        scenario_list = []
        for id_ego in self._agents_track_infos.keys():
            ego_track_info = self.__get_agent_track_info__(id_ego)
            new_scenario = ScenarioTrackInfo(
                track_filename=self._track_filename, ego_track_info=ego_track_info, xy_offset=self._xy_offset)

            ids_others = self.__find_other_ids__(id_ego)
            for id_o in ids_others:
                agent_o_track_info = self.__get_agent_track_info__(id_o)
                new_scenario.AddTrackInfoOtherAgent(agent_o_track_info)

            scenario_list.append(new_scenario)

        return scenario_list

    def decompose(self):

        scenario_list = self.__collect_all_scenario_infos__()
        return scenario_list
