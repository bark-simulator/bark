# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT


import unittest
import os
from modules.runtime.scenario.scenario_generation.uniform_vehicle_distribution\
  import UniformVehicleDistribution
from modules.runtime.scenario.scenario_generation.scenario_generation\
  import ScenarioGeneration

from modules.runtime.scenario.scenario_generation.configurable_scenario_generation \
  import ConfigurableScenarioGeneration
from modules.runtime.scenario.scenario_generation.drone_challenge\
  import DroneChallengeScenarioGeneration

from bark.geometry import *


class ScenarioGenerationTests(unittest.TestCase):
  @unittest.skip
  def test_configurable_scenario_generation_default_params(self):
    scenario_generation = ConfigurableScenarioGeneration(num_scenarios=2)
    scenario_generation.dump_scenario_list("test.scenario")

    scenario_loader = ScenarioGeneration()
    scenario_loader.load_scenario_list("test.scenario")

    self.assertEqual(len(scenario_loader._scenario_list), 2)
    self.assertEqual(len(scenario_loader._scenario_list[0]._agent_list), len(scenario_generation._scenario_list[0]._agent_list))

  def test_find_overlaps_configurable_scenario_generation(self):
    shape = Polygon2d([0, 0, 0], [Point2d(-1,0),
                      Point2d(-1,1),
                      Point2d(1,1),
                      Point2d(1,0)])

    agent_states1 = [[0, 1, 0, 0, 0], [0, 4, 0, 0, 0], [0, 8, 0, 0, 0]] # agents along x axis
    agent_geometries1 = [shape, shape, shape]

    agent_states2 = [[0, 4, -10, 0, 0], [0, 4, 0, 0, 0], [0, 4, 20, 0, 0]] # agents along y axis at x= 4
    agent_geometries2 = [shape, shape, shape]

    collected_sources_sinks_agent_states_geometries = [(agent_states1, agent_geometries1),
                                                  (agent_states2, agent_geometries2)]
    
    overlaps = ConfigurableScenarioGeneration.find_overlaps_in_sources_sinks_agents( 
                  collected_sources_sinks_agent_states_geometries)


    self.assertTrue("0-1" in overlaps)


      

    
    

if __name__ == '__main__':
  unittest.main()

