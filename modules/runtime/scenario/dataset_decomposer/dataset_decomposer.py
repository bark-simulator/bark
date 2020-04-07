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

    def __find_others__(self, id_ego):
        list_others = []
        time_ego_first = self.track_dict[id_ego].time_stamp_ms_first
        time_ego_last = self.track_dict[id_ego].time_stamp_ms_last

        for id_other in self.track_dict.keys():
            if id_ego != id_other:
                if self.track_dict[id_other].time_stamp_ms_last < time_ego_first:
                    # other ends too early
                    pass
                elif self.track_dict[id_other].time_stamp_ms_first > time_ego_last:
                    # other starts too late
                    pass
                else:
                    list_others.append(id_other)

        return list_others

    def __find_all_scenarios__(self):
        # for each agent extract ids of other agents present in the same time span
        list_others_dict = {}

        for id_ego in self.track_dict.keys():
            list_others_dict[id_ego] = self.__find_others__(id_ego)

        return list_others_dict

    def __fill_scenario_params__(self, list_others_dict, id_ego):
        params_temp = ParameterServer()
        params_temp["MapFilename"] = self.map_filename
        params_temp["TrackFilename"] = self.track_filename
        params_temp["TrackIds"] = list_others_dict[id_ego]
        params_temp["StartTs"] = self.track_dict[id_ego].time_stamp_ms_first
        params_temp["EndTs"] = self.track_dict[id_ego].time_stamp_ms_last
        params_temp["EgoTrackId"] = id_ego

        # hacking new child InteractionDatasetScenarioGeneration
        params = ParameterServer()
        params["InteractionDatasetScenarioGeneration"] = params_temp

        return params

    def decompose(self, output_dir):

        if os.path.exists(output_dir):
            shutil.rmtree(output_dir)
            os.makedirs(output_dir, exist_ok=True)

        list_others_dict = self.__find_all_scenarios__()

        # create json file
        for id_ego in list_others_dict:
            params_filename = output_dir + "/params_id_ego_" + str(id_ego)
            params = self.__fill_scenario_params__(list_others_dict, id_ego)
            params.save(params_filename + ".json")
