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
from modules.runtime.commons.parameters import ParameterServer
from modules.runtime.runtime import Runtime
from modules.runtime.viewer.matplotlib_viewer import MPViewer
from bark.models.behavior import BehaviorModel

class DummyBehaviorModel(BehaviorModel):
  """Dummy Python behavior model
  """
  def __init__(self,
               params=None):
    BehaviorModel.__init__(self, params)
    self._params = params

  def plan(self, delta_time, world):
    # return trajectory
    # time, x, y, theta, vel
    traj = np.array([[0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0]])

    # TODO(@hart): set last trajectory
    super(DummyBehaviorModel, self).set_last_action(0)
    super(DummyBehaviorModel, self).set_last_trajectory(traj)
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
    env._world.get_agent(0).behavior_model = behavior_model
    env._world.get_agent(0).behavior_model.clone()
    print(env._world.get_agent(0).behavior_model.plan(0.1, env._world))

    env.reset()
    env._world.get_agent(0).behavior_model = behavior_model
    env._world.step(0.2)

    # for i in range(0, 5):
    #   print(i)
    #   env.step()


if __name__ == '__main__':
  unittest.main()