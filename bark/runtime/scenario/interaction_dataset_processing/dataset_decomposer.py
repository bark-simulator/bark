# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import os
import shutil

from bark.runtime.commons.parameters import ParameterServer
from bark.runtime.scenario.interaction_dataset_processing.interaction_dataset_reader import TrajectoryFromTrack
from bark.runtime.scenario.interaction_dataset_processing.agent_track_info import AgentTrackInfo
from bark.runtime.scenario.interaction_dataset_processing.scenario_track_info import ScenarioTrackInfo
from bark.runtime.scenario.scenario import Scenario
from bark.core.geometry import Point2d, Collide

from com_github_interaction_dataset_interaction_dataset.python.utils import dataset_reader



class DatasetDecomposer:
    def __init__(self, map_filename, track_filename):
        self._map_filename = map_filename
        self._track_filename = track_filename
        self._track_dict = dataset_reader.read_tracks(track_filename)
        self._map_interface = self.__setup_map_interface__()
        self._agents_track_infos = self.__setup_agents_track_infos__()

    def __setup_map_interface__(self):
        params = ParameterServer()
        # we are creating a dummy scenario to get the map interface from it
        scenario = Scenario(map_file_name=self._map_filename,
                            json_params=params.ConvertToDict())
        world = scenario.GetWorldState()
        map_interface = world.map
        return map_interface

    def __find_first_ts_on_map__(self, id_ego):
        traj = TrajectoryFromTrack(self._track_dict[id_ego])
        for state in traj:
            point_agent = Point2d(state[1], state[2])
            lane_list = self._map_interface.find_nearest_lanes(point_agent, 3)
            for lane in lane_list:
                polygon = self._map_interface.GetRoadgraph().GetLanePolygonForLaneId(lane.lane_id)
                if Collide(polygon, point_agent):
                    time_ego_first = state[0]*1e3  # use timestamp in ms
                    return time_ego_first

        return None

    def __setup_agents_track_infos__(self):
        # dictionary mapping first valid timestamp to agnet id
        agents_track_infos = {}
        for agent_id in self._track_dict.keys():
            # TODO: this could be made optional
            first_ts_on_map = self.__find_first_ts_on_map__(agent_id)
            if first_ts_on_map is None:
                print("Agent %d not found on map" % agent_id)
                pass
            else:
                start_offset = first_ts_on_map
                end_offset = self._track_dict[agent_id].time_stamp_ms_last
                new_agent = AgentTrackInfo(filename=self._track_filename, track_id=agent_id,
                                           start_offset=start_offset, end_offset=end_offset)
                agents_track_infos[agent_id] = new_agent
        return agents_track_infos

    def __get_agent_track_info__(self, agent_id):
        if agent_id in self._agents_track_infos.keys():
            return self._agents_track_infos[agent_id]
        else:
            raise ValueError("agent {} not available".format(agent_id))

    def __find_all_ids__(self, id_ego):
        # for each agent extract ids of other agents present in the same time span
        list_ids = []
        time_ego_first = self.__get_agent_track_info__(id_ego).GetStartOffset()
        time_ego_last = self.__get_agent_track_info__(id_ego).GetEndOffset()

        for id_current in self._agents_track_infos.keys():
            if id_ego == id_current:
                pass
            elif self.__get_agent_track_info__(id_current).GetEndOffset() < time_ego_first:
                # other ends too early
                pass
            elif self.__get_agent_track_info__(id_current).GetStartOffset() > time_ego_last:
                # other starts too late
                pass
            else:
                list_ids.append(id_current)

        return list_ids

    def __find_all_scenarios__(self):
        scenario_list = []
        for id_ego in self._agents_track_infos.keys():
            ego_track_info = self.__get_agent_track_info__(id_ego)
            new_scenario = ScenarioTrackInfo(
                map_filename=self._map_filename, track_filename=self._track_filename, ego_track_info=ego_track_info)

            ids_others = self.__find_all_ids__(id_ego)
            for id_o in ids_others:
                agent_o_track_info = self.__get_agent_track_info__(id_o)
                new_scenario.AddTrackInfoOtherAgent(agent_o_track_info)

            scenario_list.append(new_scenario)

        return scenario_list

    def decompose(self):

        scenario_list = self.__find_all_scenarios__()
        return scenario_list
