# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import numpy as np
import time
from modules.runtime.commons.parameters import ParameterServer
from modules.runtime.viewer.matplotlib_viewer import MPViewer
from modules.runtime.scenario.scenario_generation.config_with_ease import \
  LaneCorridorConfig, ConfigWithEase
from modules.runtime.runtime import Runtime
from modules.runtime.viewer.panda3d_easy import Panda3dViewer


# parameters
param_server = ParameterServer()

# configure both lanes of the highway. the right lane has one controlled agent
left_lane = LaneCorridorConfig(params=param_server,
                               road_ids=[16],
                               lane_corridor_id=0)
right_lane = LaneCorridorConfig(params=param_server,
                                road_ids=[16],
                                lane_corridor_id=1,
                                controlled_ids=True)

# create 5 scenarios
scenarios = \
  ConfigWithEase(num_scenarios=5,
                 map_file_name="modules/runtime/tests/data/city_highway_straight.xodr",
                 random_seed=0,
                 params=param_server,
                 lane_corridor_configs=[left_lane, right_lane])

# viewer
viewer = MPViewer(params=param_server,
                  x_range=[-40, 40],
                  y_range=[-40, 40],
                  follow_agent_id=True)

# gym like interface
env = Runtime(step_time=0.2,
              viewer=viewer,
              scenario_generator=scenarios,
              render=True)
      
sim_step_time = param_server["simulation"]["step_time",
                                          "Step-time used in simulation",
                                          0.05]
sim_real_time_factor = param_server["simulation"]["real_time_factor",
                                                  "execution in real-time or faster",
                                                  0.5]

# run 3 scenarios
for episode in range(0, 3):
  env.reset()
  # step each scenario 20 times
  for step in range(0, 10):
    env.step()
    time.sleep(sim_step_time/sim_real_time_factor)
