# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import unittest
from bark.runtime.commons.parameters import ParameterServer
from bark.runtime.scenario.interaction_dataset_processing.dataset_decomposer import DatasetDecomposer
from bark.runtime.scenario import Scenario
import os
from bark.core.world.opendrive import XodrDrivingDirection


def create_map_interface(map_file_name, road_ids):
    params = ParameterServer()
    # we are creating a dummy scenario to get the map interface from it
    scenario = Scenario(map_file_name=map_file_name,
                        json_params=params.ConvertToDict())
    world = scenario.GetWorldState()
    map_interface = world.map
    map_interface.GenerateRoadCorridor(road_ids, XodrDrivingDirection.forward)
    return map_interface
    
class DatasetDecomposerTest(unittest.TestCase):
    def test_decompose_dataset(self):

        map_filename = os.path.join(os.path.dirname(__file__),"data/DR_DEU_Merging_MT_v01_shifted.xodr")
        track_filename = os.path.join(os.path.dirname(__file__),"data/interaction_dataset_DEU_Merging_dummy_track.csv")
        xy_offset = [0, 0]
        road_ids = [0,1]
        vehicle_length_max=5

        map_interface = create_map_interface(map_filename, road_ids)
        road_corridor = map_interface.GetRoadCorridor(road_ids, XodrDrivingDirection.forward)

        dataset_decomposer = DatasetDecomposer(
            map_interface=map_interface, road_corridor=road_corridor, track_filename=track_filename, vehicle_length_max=vehicle_length_max, xy_offset=xy_offset)

        scenario_list = dataset_decomposer.decompose()
        self.assertEqual(len(scenario_list), 2)
        for scen in scenario_list:
          self.assertEqual(int(scen._start_ts), scen._start_ts)
          self.assertEqual(int(scen._end_ts), scen._end_ts)

    def test_decompose_dataset_max_length(self):

        map_filename = os.path.join(os.path.dirname(__file__),"data/DR_DEU_Merging_MT_v01_shifted.xodr")
        track_filename = os.path.join(os.path.dirname(__file__),"data/interaction_dataset_DEU_Merging_dummy_track.csv")
        xy_offset = [0, 0]
        road_ids = [0,1]
        vehicle_length_max=1

        map_interface = create_map_interface(map_filename, road_ids)
        road_corridor = map_interface.GetRoadCorridor(road_ids, XodrDrivingDirection.forward)

        dataset_decomposer = DatasetDecomposer(
            map_interface=map_interface, road_corridor=road_corridor, track_filename=track_filename, vehicle_length_max=vehicle_length_max, xy_offset=xy_offset)

        scenario_list = dataset_decomposer.decompose()
        self.assertEqual(len(scenario_list), 0)

    def test_decompose_dataset_centered(self):

        map_filename = os.path.join(os.path.dirname(__file__),"data/DR_DEU_Merging_MT_v01_centered.xodr")
        track_filename = os.path.join(os.path.dirname(__file__),"data/interaction_dataset_DEU_Merging_dummy_track.csv")
        xy_offset = [-900, -900]
        road_ids = [0,1]
        vehicle_length_max=5

        map_interface = create_map_interface(map_filename, road_ids)
        road_corridor = map_interface.GetRoadCorridor(road_ids, XodrDrivingDirection.forward)

        dataset_decomposer = DatasetDecomposer(
            map_interface=map_interface, road_corridor=road_corridor, track_filename=track_filename, vehicle_length_max=vehicle_length_max, xy_offset=xy_offset)

        scenario_list = dataset_decomposer.decompose()
        self.assertEqual(len(scenario_list), 2)
        for scen in scenario_list:
          self.assertEqual(int(scen._start_ts), scen._start_ts)
          self.assertEqual(int(scen._end_ts), scen._end_ts)

if __name__ == '__main__':
    unittest.main()

