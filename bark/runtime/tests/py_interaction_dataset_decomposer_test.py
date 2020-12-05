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
import os

class DatasetDecomposerTest(unittest.TestCase):
    def test_decompose_dataset(self):

        map_filename = os.path.join(os.path.dirname(__file__),"data/DR_DEU_Merging_MT_v01_shifted.xodr")
        track_filename = os.path.join(os.path.dirname(__file__),"data/interaction_dataset_DEU_Merging_dummy_track.csv")
        xy_offset = [0, 0]

        dataset_decomposer = DatasetDecomposer(
            map_filename=map_filename, track_filename=track_filename, xy_offset=xy_offset)

        scenario_list = dataset_decomposer.decompose()
        for scen in scenario_list:
          self.assertEqual(int(scen._start_ts), scen._start_ts)
          self.assertEqual(int(scen._end_ts), scen._end_ts)

    def test_decompose_dataset_centered(self):

        map_filename = os.path.join(os.path.dirname(__file__),"data/DR_DEU_Merging_MT_v01_centered.xodr")
        track_filename = os.path.join(os.path.dirname(__file__),"data/interaction_dataset_DEU_Merging_dummy_track.csv")
        xy_offset = [-900, -900]

        dataset_decomposer = DatasetDecomposer(
            map_filename=map_filename, track_filename=track_filename, xy_offset=xy_offset)

        scenario_list = dataset_decomposer.decompose()
        for scen in scenario_list:
          self.assertEqual(int(scen._start_ts), scen._start_ts)
          self.assertEqual(int(scen._end_ts), scen._end_ts)

if __name__ == '__main__':
    unittest.main()

