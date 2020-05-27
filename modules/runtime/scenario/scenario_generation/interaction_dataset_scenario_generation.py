# Copyright (c) 2020 Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

from modules.runtime.scenario.scenario import Scenario
from modules.runtime.scenario.scenario_generation.scenario_generation \
    import ScenarioGeneration
from modules.runtime.scenario.interaction_dataset_processing.interaction_dataset_reader import agent_from_trackfile
from modules.runtime.commons.parameters import ParameterServer
# PyBind imports
from bark.world.map import *
from bark.models.dynamic import *
from bark.models.execution import *
import os.path


class InteractionDatasetScenarioGeneration(ScenarioGeneration):

    def __init__(self, params=None, num_scenarios=None, random_seed=None):
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
        self.start_time = params_temp["StartTs",
            "Timestamp when to start the scenario (ms)", 0]
        self.end_time = params_temp["EndTs",
            "Timestamp when to end the scenario (ms)", None]
        self.ego_track_id = params_temp["EgoTrackId", "TrackID of ego", -1]
        self.behavior_models = params_temp["BehaviorModel",
            "Overwrite static trajectory with prediction model", {}]

    # TODO: remove code duplication with configurable scenario generation
    def create_scenarios(self, params, num_scenarios):
        """ 
            see baseclass
        """
        scenario_list = []
        for scenario_idx in range(0, num_scenarios):
          scenario = self.create_single_scenario()     
          scenario_list.append(scenario)
        return scenario_list

    def create_single_scenario(self):
        scenario = Scenario(map_file_name=self._map_file_name,
                            json_params=self._params.ConvertToDict())
        world = scenario.get_world_state()
        agent_list = []
        eval_agent_ids = []
        track_params = ParameterServer()
        track_params["filename"] = self._track_file_name
        track_params["execution_model"] = 'ExecutionModelInterpolate'
        track_params["dynamic_model"] = 'SingleTrackModel'
        track_params["map_interface"] = world.map
        track_params["start_offset"] = self.start_time
        track_params["end_offset"] = self.end_time
        for track_id in self._track_ids:
            track_params["track_id"] = track_id
            if str(track_id) in self.behavior_models:
                track_params["behavior_model"] = self.behavior_models[str(track_id)]
            else:
                track_params["behavior_model"] = None
            agent = agent_from_trackfile(track_params, self._params, track_id)
            agent_list.append(agent)
            if track_id == self.ego_track_id:
                eval_agent_ids = [agent.id]
        scenario._agent_list = agent_list

        scenario._eval_agent_ids = eval_agent_ids
        return scenario
