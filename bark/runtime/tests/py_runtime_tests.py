# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT


import unittest
import os
import numpy as np
from bark.runtime.scenario.scenario_generation.deterministic \
  import DeterministicScenarioGeneration
from bark.runtime.scenario.scenario_generation.scenario_generation \
  import ScenarioGeneration
from bark.core.world.goal_definition import GoalDefinition, GoalDefinitionPolygon
from bark.runtime.commons.parameters import ParameterServer
from bark.runtime.runtime import Runtime
from bark.runtime.viewer.matplotlib_viewer import MPViewer

class ScenarioGenerationTests(unittest.TestCase):
  def test_uniform_vehicle_distribution_default_params(self):
    param_server = ParameterServer(
      filename=os.path.join(os.path.dirname(__file__),"data/deterministic_scenario_test.json"))
    
    mapfile = os.path.join(os.path.dirname(__file__),"data/city_highway_straight.xodr")
    param_server["Scenario"]["Generation"]["DeterministicScenarioGeneration"]["MapFilename"] = mapfile
    scenario_generation = DeterministicScenarioGeneration(num_scenarios=2,
                                                          random_seed=0,
                                                          params=param_server)
    scenario_generation.dump_scenario_list("test.scenario")
    self.assertEqual(len(scenario_generation._scenario_list), 2)
    self.assertEqual(len(scenario_generation._scenario_list[0]._agent_list), 2)

    # assert scenario generation
    agent0 = scenario_generation._scenario_list[0]._agent_list[0]
    np.testing.assert_array_equal(agent0.state, np.array([0., 0., 0., 0., 5.0]))
    self.assertEqual(agent0.id, 0)
    self.assertEqual(str(agent0.behavior_model), "bark.behavior.BehaviorConstantVelocity")
    self.assertEqual(str(agent0.dynamic_model), "bark.dynamic.SingleTrackModel")
    self.assertEqual(str(agent0.execution_model), "bark.dynamic.ExecutionModelInterpolate")
    agent1 = scenario_generation._scenario_list[0]._agent_list[1]
    #np.testing.assert_array_equal(agent1.state, np.array([0., 10., 0., 0., 5.0]))
    self.assertEqual(agent1.id, 1)
    self.assertEqual(str(agent1.behavior_model), "bark.behavior.BehaviorIDMClassic")
    
    # TODO(@hart): make sure the map is not reloaded for rl-performance
    for _ in range(0, 10):
      _ = scenario_generation.get_next_scenario()

    # loading serialized scenarios
    scenario_loader = ScenarioGeneration()
    scenario_loader.load_scenario_list("test.scenario")
    self.assertEqual(len(scenario_loader._scenario_list), 2)
    self.assertEqual(len(scenario_loader._scenario_list[0]._agent_list),
                     len(scenario_generation._scenario_list[0]._agent_list))
    self.assertEqual(
      str(scenario_loader._scenario_list[0]._agent_list[0].behavior_model),
      "bark.behavior.BehaviorConstantVelocity")
    #np.testing.assert_array_equal(scenario_loader._scenario_list[0]._agent_list[0].state,
    #  np.array([0., 0., 0., 0., 5.0]))

    self.assertEqual(
      str(scenario_loader._scenario_list[0]._agent_list[1].behavior_model),
      "bark.behavior.BehaviorIDMClassic")
    #np.testing.assert_array_equal(scenario_loader._scenario_list[0]._agent_list[1].state,
    #  np.array([0., 10., 0., 0., 5.0]))


if __name__ == '__main__':
  unittest.main()