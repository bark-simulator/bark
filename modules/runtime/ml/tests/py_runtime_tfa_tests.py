# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT


import unittest
import os
from modules.runtime.scenario.scenario_generation.uniform_vehicle_distribution import UniformVehicleDistribution
from modules.runtime.ml.runtime_rl import RuntimeRL
from modules.runtime.ml.tfa_wrapper import TFAWrapper
from modules.runtime.ml.nn_state_observer import StateConcatenation
from modules.runtime.ml.action_wrapper import MotionPrimitives
from modules.runtime.ml.state_evaluator import GoalReached
from modules.runtime.commons.parameters import ParameterServer
from modules.runtime.viewer.matplotlib_viewer import MPViewer
import numpy as np

# tfa
from tf_agents.environments import tf_py_environment


class RuntimeRLTests(unittest.TestCase):
    def test_motion_primitives_concat_state(self):
        params = ParameterServer(filename="modules/runtime/tests/data/highway_merging.json")
        scenario_generation = UniformVehicleDistribution(num_scenarios=3, random_seed=0, params=params)
        state_observer = StateConcatenation(params=params)
        action_wrapper = MotionPrimitives(params=params)
        evaluator = GoalReached(params=params)
        viewer = MPViewer(params=params, x_range=[-30,30], y_range=[-20,40], follow_agent_id=True) #use_world_bounds=True) # 

        runtimerl = RuntimeRL(action_wrapper=action_wrapper, nn_observer=state_observer,
                              evaluator=evaluator, step_time=0.05, viewer=viewer,
                              scenario_generator=scenario_generation)

        tfa_env = TFAWrapper(runtimerl)
        time_step = tfa_env.reset()
        utils.validate_py_environment(tfa_env, episodes=5)
        train_env = tf_py_environment.TFPyEnvironment(tfa_env)

if __name__ == '__main__':
    unittest.main()