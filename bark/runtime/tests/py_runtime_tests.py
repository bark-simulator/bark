# Copyright (c) 2021 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import unittest
import os
import numpy as np
import time
from bark.runtime.commons.parameters import ParameterServer
from bark.runtime.viewer.matplotlib_viewer import MPViewer
from bark.runtime.viewer.buffered_viewer import BufferedViewer
from bark.runtime.scenario.scenario_generation.config_with_ease import \
  LaneCorridorConfig, ConfigWithEase
from bark.runtime.runtime import Runtime
from bark.examples.paths import Data

from bark.core.world.opendrive import *
from bark.core.world.goal_definition import *
from bark.core.models.behavior import *
from bark.core.commons import SetVerboseLevel

# scenario
class CustomLaneCorridorConfig(LaneCorridorConfig):
  def __init__(self,
               params=None,
               **kwargs):
    super(CustomLaneCorridorConfig, self).__init__(params, **kwargs)
  
  def goal(self, world):
    road_corr = world.map.GetRoadCorridor(
      self._road_ids, XodrDrivingDirection.forward)
    lane_corr = self._road_corridor.lane_corridors[0]
    return GoalDefinitionPolygon(lane_corr.polygon)


def GenerateRuntime():
    # parameters
    param_server = ParameterServer()
    # configure both lanes of the highway. the right lane has one controlled agent
    left_lane = CustomLaneCorridorConfig(params=param_server,
                                        lane_corridor_id=0,
                                        road_ids=[0, 1],
                                        behavior_model=BehaviorMobilRuleBased(param_server),
                                        s_min=5.,
                                        s_max=50.)
    right_lane = CustomLaneCorridorConfig(params=param_server,
                                          lane_corridor_id=1,
                                          road_ids=[0, 1],
                                          controlled_ids=True,
                                          behavior_model=BehaviorMobilRuleBased(param_server),
                                          s_min=5.,
                                          s_max=20.)
    scenarios = \
      ConfigWithEase(num_scenarios=3,
                    map_file_name=Data.xodr_data("DR_DEU_Merging_MT_v01_shifted"),
                    random_seed=0,
                    params=param_server,
                    lane_corridor_configs=[left_lane, right_lane])
    # viewer
    viewer = MPViewer(params=param_server,
                      x_range=[-35, 35],
                      y_range=[-35, 35],
                      follow_agent_id=True)
    env = Runtime(step_time=0.2,
                  viewer=viewer,
                  scenario_generator=scenarios,
                  render=True,
                  maintain_world_history=True)
    return env

class PyRuntimeTest(unittest.TestCase):
  # def test_general_runtime(self):
  #   env = GenerateRuntime()
  #   env.reset()
  #   for step in range(0, 5):
  #     env.step()
  
  def test_buffered_viewer(self):
    env = GenerateRuntime()
    buffered_viewer = BufferedViewer()
    env._viewer = buffered_viewer
    env.reset()
    for step in range(0, 5):
      env.step()
      
if __name__ == '__main__':
  unittest.main()