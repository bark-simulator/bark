# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT


import unittest
import os
from modules.runtime.scenario.scenario_generation.uniform_vehicle_distribution import UniformVehicleDistribution
from modules.runtime.ml.runtime_rl import RuntimeRL
from modules.runtime.viewer.pygame_viewer import PygameViewer


class RuntimeRLTests(unittest.TestCase):
    def test_motion_primitives_concat_state(self):
        scenario_generation = UniformVehicleDistribution(num_scenarios=1, random_seed=0)

        runtimerl = RuntimeRL(action_wrapper=None, nn_observer=None,
                        reward_observer=None, step_time=0.2, viewer=PygameViewer,
                        scenario_generator=scenario_generation)

        runtimerl.reset()

        next_state, reward, done, success = runtimerl.step()
        
        
if __name__ == '__main__':
    unittest.main()