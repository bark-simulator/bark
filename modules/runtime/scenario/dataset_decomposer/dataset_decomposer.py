# Copyright (c) 2020 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import os
import shutil

from modules.runtime.commons.parameters import ParameterServer
from modules.runtime.scenario.scenario_generation.interaction_dataset_reader import agent_from_trackfile

from com_github_interaction_dataset_interaction_dataset.python.utils import dataset_reader


class DatasetDecomposer:
    def __init__(self, map_filename, track_filename):
        self.map_filename = map_filename
        self.track_filename = track_filename
        self.track_dict = dataset_reader.read_tracks(track_filename)

    def __find_all_ids__(self, id_ego):
        list_ids = []
        time_ego_first = self.track_dict[id_ego].time_stamp_ms_first
        time_ego_last = self.track_dict[id_ego].time_stamp_ms_last

        for id_current in self.track_dict.keys():
            if self.track_dict[id_current].time_stamp_ms_last < time_ego_first:
                # other ends too early
                pass
            elif self.track_dict[id_current].time_stamp_ms_first > time_ego_last:
                # other starts too late
                pass
            else:
                list_ids.append(id_current)

        return list_ids

    def __find_all_scenarios__(self):
        # for each agent extract ids of other agents present in the same time span
        list_track_dict = {}

        for id_ego in self.track_dict.keys():
            list_track_dict[id_ego] = self.__find_all_ids__(id_ego)

        return list_track_dict

    def __fill_dict_scenario__(self, list_others_dict, id_ego):

        dict_scenario = {}
        dict_scenario["MapFilename"] = self.map_filename
        dict_scenario["TrackFilename"] = self.track_filename
        dict_scenario["TrackIds"] = list_others_dict[id_ego]
        dict_scenario["StartTs"] = self.track_dict[id_ego].time_stamp_ms_first
        dict_scenario["EndTs"] = self.track_dict[id_ego].time_stamp_ms_last
        dict_scenario["EgoTrackId"] = id_ego

        return dict_scenario

    def decompose(self):

        list_track_dict = self.__find_all_scenarios__()

        dict_scenario_list = []
        for id_ego in list_track_dict:
            dict_scenario_list.append(
                self.__fill_dict_scenario__(list_track_dict, id_ego))

        return dict_scenario_list
