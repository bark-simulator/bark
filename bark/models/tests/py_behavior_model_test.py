# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.


import unittest
import os
import numpy as np
from bark.runtime.scenario.scenario_generation.deterministic \
  import DeterministicScenarioGeneration
from bark.runtime.scenario.scenario_generation.scenario_generation \
  import ScenarioGeneration

from bark.core.world.goal_definition import GoalDefinition, GoalDefinitionPolygon
from bark.core.geometry import *
from bark.core.world import World
from bark.runtime.commons.parameters import ParameterServer
from bark.runtime.runtime import Runtime
from bark.runtime.viewer.matplotlib_viewer import MPViewer
from bark.core.models.behavior import BehaviorModel, BehaviorDynamicModel
from bark.core.models.dynamic import SingleTrackModel


class PythonBehaviorModelWrapper(BehaviorModel):
  """Dummy Python behavior model
  """
  def __init__(self,
               dynamic_model = None,
               params = None):
    # BehaviorDynamicModel.__init__(self, dynamic_model, params)
    BehaviorModel.__init__(self, params)
    self._dynamic_model = dynamic_model
    self._params = params

  def Plan(self, delta_time, world):
    super(PythonBehaviorModelWrapper, self).ActionToBehavior(
      np.array([2., 1.], dtype=np.float32))
    # print(super(PythonBehaviorModelWrapper, self).GetAction())
    trajectory = np.array([[0., 0., 0., 0., 0.],
                           [0., 0., 0., 0., 0.]], dtype=np.float32)
    super(PythonBehaviorModelWrapper, self).SetLastTrajectory(trajectory)
    return trajectory

  def Clone(self):
    return self


class PythonBehaviorModelWrapperInheritance(BehaviorModel):
  """Dummy Python behavior model
  """
  def __init__(self,
               params = None):
    BehaviorModel.__init__(
      self, params)
    self._dynamic_behavior_model = BehaviorDynamicModel(params)
  
  def Plan(self, delta_time, world):
    self._dynamic_behavior_model.ActionToBehavior(
      np.array([2., 1.], dtype=np.float32))
    trajectory = self._dynamic_behavior_model.Plan(delta_time, world)
    super(PythonBehaviorModelWrapperInheritance, self).SetLastTrajectory(trajectory)
    return trajectory

  def Clone(self):
    return self


class PyBehaviorModelTests(unittest.TestCase):
  def test_python_model(self):
    param_server = ParameterServer(
      filename= os.path.join(os.path.dirname(__file__),"../../runtime/tests/data/deterministic_scenario.json"))
    param_server
    
    mapfile = os.path.join(os.path.dirname(__file__),"../../runtime/tests/data/city_highway_straight.xodr")
    param_server["Scenario"]["Generation"]["DeterministicScenarioGeneration"]["MapFilename"] = mapfile
    scenario_generation = DeterministicScenarioGeneration(num_scenarios=3,
                                                          random_seed=0,
                                                          params=param_server)
    viewer = MPViewer(params=param_server,
                      follow_agent_id=False,
                      use_world_bounds=True)
    scenario, idx = scenario_generation.get_next_scenario()
    world = scenario.GetWorldState()
    single_track_model = SingleTrackModel(param_server)
    behavior_model = PythonBehaviorModelWrapper(
      single_track_model, param_server)
    world.GetAgent(0).behavior_model = behavior_model
    world.GetAgent(0).behavior_model.SetLastAction(
      np.array([1., 1.], dtype=np.float32))
    world.Step(0.2)

  def test_python_model_inheritance(self):
    param_server = ParameterServer(
      filename= os.path.join(os.path.dirname(__file__),"../../runtime/tests/data/deterministic_scenario.json"))
    mapfile = os.path.join(os.path.dirname(__file__),"../../runtime/tests/data/city_highway_straight.xodr")
    param_server["Scenario"]["Generation"]["DeterministicScenarioGeneration"]["MapFilename"] = mapfile
    
    scenario_generation = DeterministicScenarioGeneration(num_scenarios=3,
                                                          random_seed=0,
                                                          params=param_server)
    viewer = MPViewer(params=param_server,
                      follow_agent_id=False,
                      use_world_bounds=True)
    scenario, idx = scenario_generation.get_next_scenario()
    world = scenario.GetWorldState()
    behavior_model = PythonBehaviorModelWrapperInheritance(param_server)
    
    world.GetAgent(0).behavior_model = behavior_model
    world.GetAgent(0).behavior_model.SetLastAction(
      np.array([1., 1.], dtype=np.float32))
    world.Step(0.2)


if __name__ == '__main__':
  unittest.main()