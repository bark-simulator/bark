# Copyright (c) 2020 fortiss GmbH
#
# Based on the implementation by Luis Gressenbuch
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

from modules.runtime.scenario.scenario import Scenario
from modules.runtime.scenario.scenario_generation.scenario_generation \
    import ScenarioGeneration
from modules.runtime.scenario.interaction_dataset_processing.interaction_dataset_reader import agent_from_trackfile
from modules.runtime.scenario.interaction_dataset_processing.dataset_decomposer import DatasetDecomposer
from modules.runtime.commons.parameters import ParameterServer
# PyBind imports
from bark.world.map import *
from bark.models.dynamic import *
from bark.models.execution import *


class InteractionDatasetScenarioGenerationFull(ScenarioGeneration):
  # This class reads in a track file from the interaction dataset
  # and generates a scenario for each agent as the eval agent.

    def __init__(self, params=None, num_scenarios=None, random_seed=None):
        super().__init__(params, num_scenarios, random_seed)
        self.initialize_params(params)

    def initialize_params(self, params):
        super().initialize_params(params)
        params_temp = \
            self._params["Scenario"]["Generation"]["InteractionDatasetScenarioGenerationFull"]
        self._map_file_name = params_temp["MapFilename",
                                          "Path to the open drive map",
                                          "modules/runtime/tests/data/DR_DEU_Merging_MT_v01_shifted.xodr"]
        self._track_file_name = params_temp["TrackFilename",
                                            "Path to track file (csv)",
                                            "modules/runtime/tests/data/interaction_dataset_dummy_track.csv"]
        self.behavior_models = params_temp["BehaviorModel",
                                           "Overwrite static trajectory with prediction model", {}]

    # TODO: remove code duplication with configurable scenario generation
    def create_scenarios(self, params, num_scenarios):
        """
            see baseclass
        """
        scenario_list = []

        dataset_decomposer = DatasetDecomposer(map_filename=self._map_file_name,
                                               track_filename=self._track_file_name)
        scenario_track_info_list = dataset_decomposer.decompose()

        # for scenario_idx in range(0, num_scenarios):
        for idx_s, scenario_track_info in enumerate(scenario_track_info_list):
            if idx_s < num_scenarios:
                try:
                    scenario = self.__create_single_scenario__(
                        scenario_track_info)
                except:
                    raise ValueError(
                        "Generation of scenario failed: {}".format(scenario_track_info))
                scenario_list.append(scenario)
            else:
                break
        return scenario_list

    def __create_single_scenario__(self, scenario_track_info):
        scenario = Scenario(map_file_name=self._map_file_name,
                            json_params=self._params.ConvertToDict())
        world = scenario.get_world_state()
        agent_list = []
        track_params = ParameterServer()
        #track_params["filename"] = self._track_file_name
        track_params["execution_model"] = 'ExecutionModelInterpolate'
        track_params["dynamic_model"] = 'SingleTrackModel'
        track_params["map_interface"] = world.map
        #track_params["start_offset"] = dict_scenario["StartTs"]
        #track_params["end_offset"] = dict_scenario["EndTs"]

        for id_other in scenario_track_info.GetOtherTrackInfos().keys():
            if str(id_other) in self.behavior_models:
                track_params["behavior_model"] = self.behavior_models[str(
                    id_other)]
            else:
                track_params["behavior_model"] = None
            agent = agent_from_trackfile(
                track_params, self._params, scenario_track_info, id_other)
            agent_list.append(agent)

        id_ego = scenario_track_info.GetEgoTrackInfo().GetTrackId()
        if str(id_ego) in self.behavior_models:
            track_params["behavior_model"] = self.behavior_models[str(id_ego)]
        else:
            track_params["behavior_model"] = None
        agent = agent_from_trackfile(
            track_params, self._params, scenario_track_info.GetEgoTrackInfo())
        agent_list.append(agent)

        scenario._agent_list = agent_list # must contain all agents!
        scenario._eval_agent_ids = [
            scenario_track_info.GetEgoTrackInfo().GetTrackId()]

        return scenario
