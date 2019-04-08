# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT


import unittest
from modules.runtime.scenario.scenario_generation.uniform_vehicle_distribution import UniformVehicleDistribution

class ScenarioGenerationTests(unittest.TestCase):
    def test_uniform_vehicle_distribution_default_params(self):
        scenario_generation = UniformVehicleDistribution(num_scenarios=100, random_seed=0)
        scenario_generation.params.save(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                  "params",
                  "uniform_vehicle_distribution_defaults.json"))
        
if __name__ == '__main__':
    unittest.main()

