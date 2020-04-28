# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import numpy as np
import time
from modules.runtime.commons.parameters import ParameterServer
from modules.runtime.viewer.matplotlib_viewer import MPViewer
from modules.runtime.commons.xodr_parser import XodrParser
from bark.models.behavior import BehaviorConstantVelocity
from bark.models.execution import ExecutionModelInterpolate
from bark.models.dynamic import SingleTrackModel
from bark.world import World
from bark.world.goal_definition import GoalDefinitionPolygon
from bark.world.agent import Agent
from bark.world.map import MapInterface
from bark.geometry.standard_shapes import CarLimousine
from bark.geometry import Point2d, Polygon2d
from modules.runtime.scenario.scenario_generation.config_with_ease import \
  LaneCorridorConfig, ConfigWithEase
from modules.runtime.runtime import Runtime


# Parameters
param_server = ParameterServer()

# Scenario
class CustomLaneCorridorConfig(LaneCorridorConfig):
  def __init__(self,
               road_ids=[16],
               lane_corridor_id=0,
               params=None):
    super(CustomLaneCorridorConfig, self).__init__(road_ids,
                                                 lane_corridor_id,
                                                 params)

  def position(self, world, min_s=10., max_s=150.):
    return super(CustomLaneCorridorConfig, self).position(world, min_s, max_s)

  def ds(self, s_min=20., s_max=35.):
    return np.random.uniform(s_min, s_max)

left_lane = CustomLaneCorridorConfig(lane_corridor_id=0, params=param_server)
right_lane = CustomLaneCorridorConfig(lane_corridor_id=1, params=param_server)

scenarios = \
  ConfigWithEase(num_scenarios=5,
                 map_file_name="modules/runtime/tests/data/city_highway_straight.xodr",
                 random_seed=0,
                 params=param_server,
                 lane_corridor_configs=[left_lane, right_lane])

mp_viewer = MPViewer(params=param_server, x_range=[5060, 5160], y_range=[5070,5150])


# gym like interface
env = Runtime(step_time=0.2,
              viewer=mp_viewer,
              scenario_generator=scenarios)
env.reset()
for _ in range(0, 10):
  env.step()