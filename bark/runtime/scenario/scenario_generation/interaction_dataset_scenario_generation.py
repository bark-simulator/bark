# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import os.path

from bark.runtime.scenario import Scenario
from bark.runtime.scenario.scenario_generation import ScenarioGeneration
from bark.runtime.scenario.interaction_dataset_processing.interaction_dataset_reader import InteractionDatasetReader
from bark.runtime.commons.model_json_conversion import ModelJsonConversion
from bark.runtime.commons import ParameterServer
# PyBind imports
from bark.core.world.map import *
from bark.core.models.dynamic import *
from bark.core.models.execution import *
from bark.runtime.scenario.interaction_dataset_processing.scenario_track_info import ScenarioTrackInfo
from bark.runtime.scenario.interaction_dataset_processing.agent_track_info import AgentTrackInfo
import os.path


class InteractionDatasetScenarioGeneration(ScenarioGeneration):

    def __init__(self, params=None, num_scenarios=None, random_seed=None):
        super().__init__(params, num_scenarios, random_seed)

    def initialize_params(self, params):
        super().initialize_params(params)
        params_temp = \
            self._params["Scenario"]["Generation"]["InteractionDatasetScenarioGeneration"]
        self._map_file_name = params_temp["MapFilename",
                                          "Path to the open drive map",
                                          os.path.expanduser('~') + "/bark-simulator/interaction_dataset_fortiss_internal/DR_DEU_Merging_MT/map/DR_DEU_Merging_MT_v01_shifted.xodr"]
        self._track_file_name = params_temp["TrackFilename",
                                            "Path to track file (csv)",
                                            os.path.expanduser('~') + "/bark-simulator/interaction_dataset_fortiss_internal/DR_DEU_Merging_MT/tracks/vehicle_tracks_013.csv"]
        self._track_ids = params_temp["TrackIds",
                                      "IDs of the vehicle tracks to import.",
                                      [1]]
        self._xy_offset = params_temp["XYOffset",
                                      "offset in x and y direction.", [0, 0]]
        self._start_time = params_temp["StartTs",
                                       "Timestamp when to start the scenario (ms)", 0]
        self._end_time = params_temp["EndTs",
                                     "Timestamp when to end the scenario (ms)", None]
        self._ego_track_id = params_temp["EgoTrackId", "TrackID of ego", -1]
        self._behavior_models = params_temp["BehaviorModel",
                                            "Overwrite static trajectory with prediction model", {}]
        self._use_rectangle_shape = params_temp["RectangleShape",
                                                "Use Rectangle vehicle shape", True]
        
        self._interaction_ds_reader = InteractionDatasetReader()

    # TODO: remove code duplication with configurable scenario generation
    def create_scenarios(self, params, num_scenarios):
        """
            see baseclass
        """

        if num_scenarios != 1:
            raise ValueError("Replay supports only 1 scenario")

        scenario_track_info = self.__fill_scenario_track_info__()
        scenario_list = [self.__create_single_scenario__(scenario_track_info)]

        return scenario_list

    def __fill_scenario_track_info__(self):
        if self._ego_track_id == -1:
            raise ValueError("No ego id has been defined")

        ego_track_info = AgentTrackInfo(filename=self._track_file_name, track_id=self._ego_track_id,
                                        start_time=self._start_time, end_time=self._end_time)
        scenario_track_info = ScenarioTrackInfo(track_filename=self._track_file_name, ego_track_info=ego_track_info, xy_offset=self._xy_offset)
        for track_id in self._track_ids:
            new_agent = AgentTrackInfo(filename=self._track_file_name, track_id=track_id,
                                       start_time=self._start_time, end_time=self._end_time)
            scenario_track_info.AddTrackInfoOtherAgent(new_agent)

        return scenario_track_info

    def __fill_agent_params__(self):
        return self._params

    def __create_single_scenario__(self, scenario_track_info):
        scenario_track_info.TimeSanityCheck()

        scenario = Scenario(map_file_name=self._map_file_name,
                            json_params=self._params.ConvertToDict())

        world = scenario.GetWorldState()
        track_params = ParameterServer()
        track_params["execution_model"] = 'ExecutionModelInterpolate'
        track_params["dynamic_model"] = 'SingleTrackModel'
        track_params["map_interface"] = world.map

        all_track_ids = list(scenario_track_info.GetOtherTrackInfos().keys())
        # also add ego id
        all_track_ids.append(
            scenario_track_info.GetEgoTrackInfo().GetTrackId())

        agent_list = []
        model_converter = ModelJsonConversion()
        for track_id in all_track_ids:
            if str(track_id) in self._behavior_models:
                behavior_params = self.__fill_agent_params__()
                behavior_model_name = self._behavior_models[str(track_id)]
                track_params["behavior_model"] = model_converter.convert_model(
                    behavior_model_name, behavior_params)
            else:
                track_params["behavior_model"] = None

            agent = self._interaction_ds_reader.AgentFromTrackfile(
                track_params, self._params, scenario_track_info, track_id, goal_def=None)
            # agent_params.Save("/tmp/agent_params_{}.json".format(track_id))
            agent_list.append(agent)

        scenario._agent_list = agent_list  # must contain all agents!
        scenario._eval_agent_ids = [
            scenario_track_info.GetEgoTrackInfo().GetTrackId()]

        return scenario
