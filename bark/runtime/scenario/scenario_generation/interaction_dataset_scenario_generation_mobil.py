# Copyright (c) 2020 Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.


from bark.runtime.commons import ParameterServer
from bark.runtime.scenario.scenario_generation.interaction_dataset_scenario_generation \
    import InteractionDatasetScenarioGeneration
import random

class DatasetScenarioGenerationMobil(InteractionDatasetScenarioGeneration):

    def __init__(self, params=None, num_scenarios=None, random_seed=None):
        super().__init__(params, num_scenarios, random_seed)

    def initialize_params(self, params):
        super().initialize_params(params)
        params_temp = self._params["Scenario"]["Generation"]["DatasetScenarioGenerationMobil"]
        self.politeness_range = params_temp["PolitenessRange",
                                            "Upper and lower bound for politeness factor of MOBIL", [0.0, 1.0]]
        self.min_dist_range = params_temp["MinimumDistanceRange",
                                          "Upper and lower bound for minimum distance of IDM", [1.0, 5.0]]
        self.randomize_ego_pos = params_temp["RandomizeEgoPos",
                                             "Randomly select initial position of ego", False]
        self.base_params_json = params_temp["BaseParams", "Initial parameters of each scenario", ParameterServer(
            log_if_default=True)].ConvertToDict()
        # Required to keep paramservers for each scenario alive.
        self.agent_params = []

    def create_scenarios(self, params, num_scenarios):
        """ 
            see baseclass
        """
        scenario_track_info = super().__fill_scenario_track_info__()

        scenario_list = []
        for scenario_idx in range(0, num_scenarios):
            scenario = self.__create_single_scenario__(scenario_track_info)
            scenario_list.append(scenario)

        return scenario_list

    def __sample_uniform__(self, parameter_range):
        if len(parameter_range) == 1:
            parameter_scalar = parameter_range
        elif len(parameter_range) > 2:
            raise ValueError("range has size {}".format(len(parameter_range)))
        else:
            parameter_scalar = random.uniform(
                parameter_range[0], parameter_range[1])

        return parameter_scalar

    def __fill_agent_params__(self):
        agent_params = ParameterServer(log_if_default=True, json=self.base_params_json)
        politeness = self.__sample_uniform__(self.politeness_range)
        agent_params["BehaviorMobilRuleBased"]["Politeness"] = politeness

        minimum_spacing = self.__sample_uniform__(self.min_dist_range)
        agent_params["BehaviorIDMClassic"]["MinimumSpacing"] = minimum_spacing
        # print("\n", agent_params.ConvertToDict())
        self.agent_params.append(agent_params)
        return agent_params
