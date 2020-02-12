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


class PythonBehaviorModelWrapper(DynamicBehaviorModel):
  """Dummy Python behavior model
  """
  def __init__(self,
               dynamic_model = None,
               params = None):
    DynamicBehaviorModel.__init__(self, dynamic_model, params)

  def Plan(self, delta_time, world):
    behavior_model = world.ego_agent.behavior_model

    # THIS DOES NOT WORK
    behavior_model.SetLastAction(
      np.array([2., 1.], dtype=np.float32))

    # should call Plan call of parent class
    return behavior_model.Plan(
      delta_time, observed_world)

  def Clone(self):
    return self


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
    scenario, idx = scenario_generation.get_next_scenario()
    world = scenario.get_world_state()
    
    single_track_model = SingleTrackModel(param_server)
    behavior_model = PythonBehaviorModelWrapper(single_track_model, param_server)

    # this should fail as there is no last action
    world.GetAgent(0).behavior_model = behavior_model
    world.GetAgent(0).behavior_model.SetLastAction(
      np.array([1., 1.], dtype=np.float32))
    world.Step(0.2)


if __name__ == '__main__':
  unittest.main()