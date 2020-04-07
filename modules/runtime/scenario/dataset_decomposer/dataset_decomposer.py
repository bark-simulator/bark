# Copyright (c) 2020 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import os
#import argparse
import shutil

from modules.runtime.commons.parameters import ParameterServer

from modules.runtime.scenario.scenario_generation.interaction_dataset_reader import agent_from_trackfile
from modules.runtime.scenario.scenario_generation.interaction_dataset_scenario_generation import \
    InteractionDatasetScenarioGeneration

from com_github_interaction_dataset_interaction_dataset.python.utils import dataset_reader
from com_github_interaction_dataset_interaction_dataset.python.utils import dict_utils

map_filename = "../interaction_dataset_fortiss_internal/DR_DEU_Merging_MT/map/DR_DEU_Merging_MT_shifted.xodr"
track_filename = "../interaction_dataset_fortiss_internal/DR_DEU_Merging_MT/tracks/vehicle_tracks_013.csv"

track_dict = dataset_reader.read_tracks(track_filename)

print(track_dict.keys())

# Track
# track_id
# agent_type
# length
# width
# time_stamp_ms_first
# time_stamp_ms_last
# motion_states

print(track_dict[1].time_stamp_ms_first)

# for each agent extract ids of other agents present in the same time span
list_others_dict = {}

for id_ego in track_dict.keys():
    list_others_dict[id_ego] = []
    time_ego_first = track_dict[id_ego].time_stamp_ms_first
    time_ego_last = track_dict[id_ego].time_stamp_ms_last

    for id_other in track_dict.keys():
        if id_ego != id_other:
            if track_dict[id_other].time_stamp_ms_last < time_ego_first:
                # other ends too early
                pass
            elif track_dict[id_other].time_stamp_ms_first > time_ego_last:
                # other starts too late
                pass
            else:
                list_others_dict[id_ego].append(id_other)

print(list_others_dict)

# prepare output directory
output_dir = "/tmp/" + map_filename.split("/")[-1].split(".")[0] + "_" \
    + track_filename.split("/")[-1].split(".")[0]

if os.path.exists(output_dir):
    shutil.rmtree(output_dir)
os.makedirs(output_dir, exist_ok=True)

# create json file
for id_ego in list_others_dict:

    params_temp = ParameterServer()
    params_temp["MapFilename"] = map_filename
    params_temp["TrackFilename"] = track_filename
    params_temp["TrackIds"] = list_others_dict[id_ego]
    params_temp["StartTs"] = track_dict[id_ego].time_stamp_ms_first
    params_temp["EndTs"] = track_dict[id_ego].time_stamp_ms_last
    params_temp["EgoTrackId"] = id_ego

    # hacking new child InteractionDatasetScenarioGeneration
    params = ParameterServer()
    params["InteractionDatasetScenarioGeneration"] = params_temp

    params_filename = output_dir + "/params_id_ego_" + str(id_ego)
    params.save(params_filename + ".json")
