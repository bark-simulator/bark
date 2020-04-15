# Copyright (c) 2020 fortiss GmbH
#
# Based on the implementation by Luis Gressenbuch
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

from modules.runtime.scenario.scenario import Scenario
from modules.runtime.scenario.scenario_generation.scenario_generation \
    import ScenarioGeneration
from modules.runtime.scenario.scenario_generation.interaction_dataset_reader import agent_from_trackfile
from modules.runtime.scenario.dataset_decomposer.dataset_decomposer import DatasetDecomposer
from modules.runtime.commons.parameters import ParameterServer
# PyBind imports
from bark.world.map import *
from bark.models.dynamic import *
from bark.models.execution import *


class InteractionDatasetScenarioGenerationFull(ScenarioGeneration):

    def __init__(self, params=None, num_scenarios=None, random_seed=None):
        super().__init__(params, num_scenarios, random_seed)
        self.initialize_params(params)

    def initialize_params(self, params):
        super().initialize_params(params)
        params_temp = \
            self._params["Scenario"]["Generation"]["InteractionDatasetScenarioGenerationFull"]
        self._map_file_name = params_temp["MapFilename",
                                          "Path to the open drive map",
                                          "modules/runtime/tests/data/DR_DEU_Merging_MT.xodr"]
        self._track_file_name = params_temp["TrackFilename",
                                            "Path to track file (csv)",
                                            "modules/runtime/tests/data/vehicle_tracks_000.csv"]
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
        dict_scen_list = dataset_decomposer.decompose()

        # TODO(@Klemens): limitieren durch num_scenarios?
        # for scenario_idx in range(0, num_scenarios):
        for idx_s, dict_scen in enumerate(dict_scen_list):
            if idx_s < num_scenarios:
                scenario = self.__create_single_scenario__(dict_scen)
                scenario_list.append(scenario)
            else:
                break
        return scenario_list

    def __create_single_scenario__(self, dict_scenario):
        scenario = Scenario(map_file_name=self._map_file_name,
                            json_params=self._params.convert_to_dict())
        world = scenario.get_world_state()
        agent_list = []
        eval_agent_ids = []
        track_params = ParameterServer()
        track_params["filename"] = self._track_file_name
        track_params["execution_model"] = 'ExecutionModelInterpolate'
        track_params["dynamic_model"] = 'SingleTrackModel'
        track_params["map_interface"] = world.map
        track_params["start_offset"] = dict_scenario["StartTs"]
        track_params["end_offset"] = dict_scenario["EndTs"]

        for track_id in dict_scenario["TrackIds"]:
            track_params["track_id"] = track_id
            if str(track_id) in self.behavior_models:
                track_params["behavior_model"] = self.behavior_models[str(
                    track_id)]
            else:
                track_params["behavior_model"] = None
            agent = agent_from_trackfile(track_params, self._params, track_id)
            agent_list.append(agent)
            if track_id == track_params["EgoTrackId"]:
                eval_agent_ids = [agent.id]

        scenario._agent_list = agent_list
        scenario._eval_agent_ids = eval_agent_ids

        return scenario
