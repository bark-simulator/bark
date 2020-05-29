# Copyright (c) 2020 Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import os
import shutil

from modules.runtime.commons.parameters import ParameterServer
from modules.runtime.scenario.interaction_dataset_processing.interaction_dataset_reader import agent_from_trackfile, trajectory_from_track

from com_github_interaction_dataset_interaction_dataset.python.utils import dataset_reader
from modules.runtime.scenario.scenario import Scenario
from bark.geometry import Point2d, Collide


class DatasetDecomposer:
    def __init__(self, map_filename, track_filename):
        self._map_filename = map_filename
        self._track_filename = track_filename
        self._track_dict = dataset_reader.read_tracks(track_filename)
        self._lane_polygon_list = self.__read_lane_polygons__()
        self._agents_first_valid_ts_ms = self.__setup_agents_first_valid_ts_ms__()

    def __read_lane_polygons__(self):
        params = ParameterServer()
        # we are creating a dummy scenario to get the map interface from it
        scenario = Scenario(map_file_name=self._map_filename,
                            json_params=params.ConvertToDict())
        world = scenario.GetWorldState()
        lane_ids = world.map.GetRoadgraph().GetAllLaneids()
        lane_polygon_list = []
        for lane_id in lane_ids:
            lane_polygon_list.append(
                world.map.GetRoadgraph().GetLanePolygonForLaneId(lane_id))
        return lane_polygon_list

    def __setup_agents_first_valid_ts_ms__(self):
        # dictionary mapping first valid timestamp to agnet id
        agents_to_ts = {}
        for agent_id in self._track_dict.keys():
            agents_to_ts[agent_id] = self.__find_first_ts_on_map__(agent_id)
        return agents_to_ts

    def __find_all_ids__(self, id_ego):
        list_ids = []
        time_ego_first = self._agents_first_valid_ts_ms[id_ego]
        time_ego_last = self._track_dict[id_ego].time_stamp_ms_last

        for id_current in self._track_dict.keys():
            if self._track_dict[id_current].time_stamp_ms_last < time_ego_first:
                # other ends too early
                pass
            elif self._track_dict[id_current].time_stamp_ms_first > time_ego_last:
                # other starts too late
                pass
            elif self._agents_first_valid_ts_ms[id_current] > time_ego_last:
                # other enter map too late
                pass
            else:
                list_ids.append(id_current)

        return list_ids

    def __find_first_ts_on_map__(self, id_ego):
        traj = trajectory_from_track(self._track_dict[id_ego])
        for state in traj:
            point_agent = Point2d(state[1], state[2])
            for poly_lane in self._lane_polygon_list:
                if Collide(poly_lane, point_agent):
                    time_ego_first = state[0]*1e3  # use timestamp in ms
                    return time_ego_first

        raise ValueError(
            "No valid time stamp in map for agent {}".format(id_ego))

    def __find_all_scenarios__(self):
            # for each agent extract ids of other agents present in the same time span
        list_track_dict = {}

        for id_ego in self._track_dict.keys():
            list_track_dict[id_ego] = self.__find_all_ids__(id_ego)

        return list_track_dict

    def __fill_dict_scenario__(self, list_others_dict, id_ego):

        dict_scenario = {}
        dict_scenario["MapFilename"] = self._map_filename
        dict_scenario["TrackFilename"] = self._track_filename
        dict_scenario["TrackIds"] = list_others_dict[id_ego]
        dict_scenario["StartTs"] = self._agents_first_valid_ts_ms[id_ego]
        dict_scenario["EndTs"] = self._track_dict[id_ego].time_stamp_ms_last
        dict_scenario["EgoTrackId"] = id_ego

        return dict_scenario

    def decompose(self):

        list_track_dict = self.__find_all_scenarios__()

        dict_scenario_list = []
        for id_ego in list_track_dict:
            dict_scenario_list.append(
                self.__fill_dict_scenario__(list_track_dict, id_ego))

        return dict_scenario_list
