# Copyright (c) 2020 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import unittest
from modules.runtime.scenario.interaction_dataset_processing.interaction_dataset_reader import agent_from_trackfile
from modules.runtime.scenario.interaction_dataset_processing.agent_track_info import AgentTrackInfo
from modules.runtime.scenario.interaction_dataset_processing.scenario_track_info import ScenarioTrackInfo
from modules.runtime.commons.parameters import ParameterServer


class InteractionDatasetReaderTest(unittest.TestCase):
    # @unittest.skip
    def test_agent_from_trackfile(self):

        track_ids = [1, 2]

        params = ParameterServer()
        params_id = params["Scenario"]["Generation"]["InteractionDataset"]
        params_id["MapFilename", "",
                  "modules/runtime/tests/data/DR_DEU_Merging_MT_v01_shifted.xodr"]
        params_id["TrackFilename", "",
                  "modules/runtime/tests/data/interaction_dataset_dummy_track.csv"]
        params_id["TrackIds", "", track_ids]
        params_id["StartTs", "", 500]
        params_id["EndTs", "", None]
        params_id["EgoTrackId", "", -1]
        params_id["BehaviorModel", "", {}]

        track_params = ParameterServer()
        track_params["filename"] = params["Scenario"]["Generation"]["InteractionDataset"]["TrackFilename"]
        track_params["execution_model"] = 'ExecutionModelInterpolate'
        track_params["dynamic_model"] = 'SingleTrackModel'
        track_params["map_interface"] = None  # world.map
        track_params["start_offset"] = params["Scenario"]["Generation"]["InteractionDataset"]["StartTs"]
        track_params["end_offset"] = params["Scenario"]["Generation"]["InteractionDataset"]["EndTs"]
        track_params["behavior_model"] = None

        agent_list = []
        for track_id in track_ids:
            track_params["track_id"] = track_id
            agent = agent_from_trackfile(track_params, params, 1)
            agent_list.append(agent)
            print(agent.behavior_model)

        assert(len(agent_list) == len(track_ids))

    def test_scenario_track_info(self):
        map_filename = "modules/runtime/tests/data/DR_DEU_Merging_MT_v01_shifted.xodr"
        track_filename = "modules/runtime/tests/data/interaction_dataset_dummy_track.csv"

        agent1 = AgentTrackInfo(
            track_filename, track_id=1, start_offset=500, end_offset=1000)
        agent2 = AgentTrackInfo(
            track_filename, track_id=2, start_offset=500, end_offset=1000)

        scen_info = ScenarioTrackInfo(
            map_filename, track_filename, agent1, start_ts=500, end_ts=1000)
        scen_info.AddTrackInfoOtherAgent(agent2)

        assert(scen_info.TimeSanityCheck() == True)


if __name__ == '__main__':
    unittest.main()
