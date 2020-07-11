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
        self.interaction_ds_reader = InteractionDatasetReader()
        super().__init__(params, num_scenarios, random_seed)
        self.initialize_params(params)

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
        self._start_time = params_temp["StartTs",
                                       "Timestamp when to start the scenario (ms)", 0]
        self._end_time = params_temp["EndTs",
                                     "Timestamp when to end the scenario (ms)", None]
        self._ego_track_id = params_temp["EgoTrackId", "TrackID of ego", -1]
        self._behavior_models = params_temp["BehaviorModel",
                                            "Overwrite static trajectory with prediction model", {}]

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
                                        start_offset=self._start_time, end_offset=self._end_time)
        scenario_track_info = ScenarioTrackInfo(
            map_filename=self._map_file_name, track_filename=self._track_file_name, ego_track_info=ego_track_info)
        for track_id in self._track_ids:
            new_agent = AgentTrackInfo(filename=self._track_file_name, track_id=track_id,
                                       start_offset=self._start_time, end_offset=self._end_time)
            scenario_track_info.AddTrackInfoOtherAgent(new_agent)

        return scenario_track_info

    def __create_single_scenario__(self, scenario_track_info):
        scenario_track_info.TimeSanityCheck()
        
        scenario = Scenario(map_file_name=self._map_file_name,
                            json_params=self._params.ConvertToDict())
        world = scenario.GetWorldState()
        agent_list = []
        track_params = ParameterServer()
        track_params["execution_model"] = 'ExecutionModelInterpolate'
        track_params["dynamic_model"] = 'SingleTrackModel'
        track_params["map_interface"] = world.map

        for id_other in scenario_track_info.GetOtherTrackInfos().keys():
            if str(id_other) in self._behavior_models:
                track_params["behavior_model"] = self._behavior_models[str(
                    id_other)]
            else:
                track_params["behavior_model"] = None
            agent = self.interaction_ds_reader.AgentFromTrackfile(
                track_params, self._params, scenario_track_info, id_other)
            agent_list.append(agent)

        id_ego = scenario_track_info.GetEgoTrackInfo().GetTrackId()
        if str(id_ego) in self._behavior_models:
            track_params["behavior_model"] = self._behavior_models[str(id_ego)]
        else:
            track_params["behavior_model"] = None
        agent = self.interaction_ds_reader.AgentFromTrackfile(
            track_params, self._params, scenario_track_info, id_ego)
        agent_list.append(agent)

        scenario._agent_list = agent_list  # must contain all agents!
        scenario._eval_agent_ids = [
            scenario_track_info.GetEgoTrackInfo().GetTrackId()]

        return scenario
