# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT


import unittest
import os
from modules.runtime.scenario.scenario_generation.deterministic \
  import DeterministicScenarioGeneration
from modules.runtime.scenario.scenario_generation.scenario_generation \
  import ScenarioGeneration
from modules.runtime.commons.parameters import ParameterServer

class ScenarioGenerationTests(unittest.TestCase):
  def test_uniform_vehicle_distribution_default_params(self):
    param_server = ParameterServer(
      filename="modules/runtime/tests/data/deterministic_scenario.json")
    scenario_generation = DeterministicScenarioGeneration(num_scenarios=2,
                                                          random_seed=0,
                                                          params=param_server)
    scenario_generation.dump_scenario_list("test.scenario")
    self.assertEqual(len(scenario_generation._scenario_list), 2)
    self.assertEqual(len(scenario_generation._scenario_list[0]._agent_list), 2)

    # TODO(@hart): assert all the agents properties as defined in the jso-file
    # TODO(@hart): make sure the map is not reloaded for rl-performance
    # TODO(@hart): make sure the scenario can be loaded and it's still the same as before
    """
    scenario_loader = ScenarioGeneration()
    scenario_loader.load_scenario_list("test.scenario")

    self.assertEqual(len(scenario_loader._scenario_list), 2)
    self.assertEqual(len(scenario_loader._scenario_list[0]._agent_list),
                     len(scenario_generation._scenario_list[0]._agent_list))
    

    # TODO(@hart): asserts
    world = scenario_loader._scenario_list[0].get_world_state()
    print(scenario_loader._scenario_list[0]._agent_list)
    print(world.agents)
    """
if __name__ == '__main__':
  unittest.main()