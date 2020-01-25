# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT


import unittest
import os
import numpy as np
from modules.runtime.scenario.scenario_generation.deterministic \
  import DeterministicScenarioGeneration
from modules.runtime.scenario.scenario_generation.scenario_generation \
  import ScenarioGeneration
from bark.world.goal_definition import GoalDefinition, GoalDefinitionPolygon
from bark.geometry import *
from bark.world import World
from modules.runtime.commons.parameters import ParameterServer
from modules.runtime.runtime import Runtime
from modules.runtime.viewer.matplotlib_viewer import MPViewer
from bark.models.behavior import BehaviorModel, DynamicBehaviorModel
from bark.models.dynamic import SingleTrackModel

class DummyBehaviorModel(BehaviorModel):
  """Dummy Python behavior model
  """
  def __init__(self,
               params=None):
    BehaviorModel.__init__(self, params)
    self._params = params

  def plan(self, delta_time, world):
    # time, x, y, theta, vel
    traj = np.array([[0.0, 5111.626, 5106.8305 + 0.0, 1.5, 10],
                     [0.2, 5111.626, 5106.8305 + 0.2, 1.5, 10],
                     [0.4, 5111.626, 5106.8305 + 0.4, 1.5, 10],
                     [0.6, 5111.626, 5106.8305 + 0.6, 1.5, 10],
                     [0.8, 5111.626, 5106.8305 + 0.8, 1.5, 10],
                     [1.0, 5111.626, 5106.8305 + 1.0, 1.5, 10],
                     [1.2, 5111.626, 5106.8305 + 1.2, 1.5, 10]])
    # this is required for the history
    super(DummyBehaviorModel, self).SetLastAction(1.23)
    super(DummyBehaviorModel, self).SetLastTrajectory(traj)
    return traj

  def clone(self):
    return DummyBehaviorModel(self._params)


class PyBehaviorModelTests(unittest.TestCase):
  def test_python_model(self):
    param_server = ParameterServer(
      filename="modules/runtime/tests/data/deterministic_scenario.json")
    scenario_generation = DeterministicScenarioGeneration(num_scenarios=3,
                                                          random_seed=0,
                                                          params=param_server)
    viewer = MPViewer(params=param_server,
                      follow_agent_id=False,
                      use_world_bounds=True)
    env = Runtime(0.2,
                  viewer,
                  scenario_generation,
                  render=True)
    
    behavior_model = DummyBehaviorModel(param_server)

    env.reset()
    env._world.GetAgent(0).behavior_model = behavior_model
    env._world.GetAgent(0).behavior_model.Clone()

    np.testing.assert_array_equal(
      env._world.GetAgent(0).behavior_model.Plan(0.2, env._world)[1],
      np.array([0.2, 5111.626, 5106.8305 + 0.2, 1.5, 10]))

    env.reset()
    env._world.GetAgent(0).behavior_model = behavior_model
    env._world.Step(0.2)
    np.testing.assert_array_equal(
      env._world.GetAgent(0).state,
      np.array([0.2, 5111.626, 5106.8305 + 0.2, 1.5, 10], dtype=np.float32))
    env._world.Step(0.2)
    np.testing.assert_array_equal(
      env._world.GetAgent(0).state,
      np.array([0.4, 5111.626, 5106.8305 + 0.4, 1.5, 10], dtype=np.float32))
    env._world.Step(0.2)
    np.testing.assert_array_equal(
      env._world.GetAgent(0).state,
      np.array([0.6, 5111.626, 5106.8305 + 0.6, 1.5, 10], dtype=np.float32))

    print("History:", env._world.GetAgent(0).history)
    # environment loop
    env.reset()
    for i in range(0, 7):
      env.Step()
    
  def test_python_model(self):
    param_server = ParameterServer(
      filename="modules/runtime/tests/data/deterministic_scenario.json")
    scenario_generation = DeterministicScenarioGeneration(num_scenarios=3,
                                                          random_seed=0,
                                                          params=param_server)
    viewer = MPViewer(params=param_server,
                      follow_agent_id=False,
                      use_world_bounds=True)
    env = Runtime(0.2,
                  viewer,
                  scenario_generation,
                  render=True)
    
    single_track_model = SingleTrackModel(param_server)
    behavior_model = DynamicBehaviorModel(single_track_model, param_server)

    env.reset()
    env._world.GetAgent(0).behavior_model = behavior_model
    env._world.GetAgent(0).behavior_model.Clone()


    env.reset()
    env._world.GetAgent(0).behavior_model = behavior_model
    env._world.GetAgent(0).behavior_model.SetLastAction(np.array([1., 2.]))
    print(env._world.GetAgent(0).behavior_model.GetLastAction())
    env._world.Step(0.2)
    print(env._world.GetAgent(0).behavior_model.GetLastAction())


if __name__ == '__main__':
  unittest.main()