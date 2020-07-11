# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import unittest
from bark.runtime.scenario.interaction_dataset_processing.interaction_dataset_reader import InteractionDatasetReader
from bark.runtime.scenario.interaction_dataset_processing.agent_track_info import AgentTrackInfo
from bark.runtime.scenario.interaction_dataset_processing.scenario_track_info import ScenarioTrackInfo
from bark.runtime.commons.parameters import ParameterServer
import os

class InteractionDatasetReaderTest(unittest.TestCase):

    def test_scenario_track_info(self):
        map_filename = os.path.join(os.path.dirname(__file__),"../runtime/tests/data/DR_DEU_Merging_MT_v01_shifted.xodr")
        track_filename = os.path.join(os.path.dirname(__file__),"../runtime/tests/data/interaction_dataset_dummy_track.csv")

        agent1 = AgentTrackInfo(
            track_filename, track_id=1, start_offset=500, end_offset=1000)

        print(agent1)

        agent2 = AgentTrackInfo(
            track_filename, track_id=2, start_offset=500, end_offset=1000)

        print(agent2)

        scen_info = ScenarioTrackInfo(
            map_filename, track_filename, agent1, start_ts=500, end_ts=1000)
        scen_info.AddTrackInfoOtherAgent(agent2)

        print(scen_info)

        assert(scen_info.TimeSanityCheck() == True)

    def test_agent_from_trackfile(self):

        map_filename = os.path.join(os.path.dirname(__file__),"../tests/data/DR_DEU_Merging_MT_v01_shifted.xodr")
        track_filename = os.path.join(os.path.dirname(__file__),"../tests/data/interaction_dataset_dummy_track.csv")

        agent_track_info = AgentTrackInfo(
            track_filename, track_id=1, start_offset=500, end_offset=1000)

        params = ParameterServer()
        params_id = params["Scenario"]["Generation"]["InteractionDataset"]
        params_id["MapFilename", "",
                  os.path.join(os.path.dirname(__file__),"../tests/data/DR_DEU_Merging_MT_v01_shifted.xodr")]
        params_id["TrackFilename", "",
                  os.path.join(os.path.dirname(__file__),"../tests/data/interaction_dataset_dummy_track.csv")]
        params_id["BehaviorModel", "", {}]

        track_params = ParameterServer()
        track_params["execution_model"] = 'ExecutionModelInterpolate'
        track_params["dynamic_model"] = 'SingleTrackModel'
        track_params["map_interface"] = None  # world.map
        track_params["behavior_model"] = None

        scenario_info = ScenarioTrackInfo(
            map_filename, track_filename, agent_track_info)
        ds_reader = InteractionDatasetReader()
        agent = ds_reader.AgentFromTrackfile(
            track_params, params, scenario_info, agent_track_info.GetTrackId())


if __name__ == '__main__':
    unittest.main()
