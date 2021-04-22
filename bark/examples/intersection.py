# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import numpy as np
import time
import os
from bark.runtime.commons.parameters import ParameterServer
from bark.runtime.viewer.matplotlib_viewer import MPViewer
from bark.runtime.viewer.video_renderer import VideoRenderer
from bark.runtime.scenario.scenario_generation.config_with_ease import \
  LaneCorridorConfig, ConfigWithEase
from bark.core.geometry.standard_shapes import CarLimousine
from bark.core.models.behavior import *
from bark.runtime.runtime import Runtime
from bark.examples.paths import Data


# Parameters Definitions
param_server = ParameterServer()

param_server["BehaviorIDMLaneTracking"]["CrosstrackErrorGain"] = 2.5
param_server["BehaviorIDMClassic"]["DesiredVelocity"] = 5.
param_server["BehaviorIntersectionRuleBased"]["BrakingDistance"] = 10.
param_server["BehaviorIntersectionRuleBased"]["PredictionTimeHorizon"] = 5.
param_server["BehaviorIntersectionRuleBased"]["AngleDiffForIntersection"] = 0.25

# configure the lane corridors and how the agents 
lane_corridors = []
lane_corridors.append(
  LaneCorridorConfig(params=param_server,
                     source_pos=[-30, -3],
                     sink_pos=[30, -3],
                     behavior_model=BehaviorIntersectionRuleBased(param_server),
                     min_vel=5.,
                     max_vel=5.,
                     ds_min=5.,
                     ds_max=10.,
                     s_min=15.,
                     s_max=30.))
lane_corridors.append(
  LaneCorridorConfig(params=param_server,
                     source_pos=[30, 3],
                     sink_pos=[-30, 3],
                     behavior_model=BehaviorIntersectionRuleBased(param_server),
                     min_vel=5.,
                     max_vel=5.,
                     ds_min=5.,
                     ds_max=10.,
                     s_min=15.,
                     s_max=30.))
lane_corridors.append(
  LaneCorridorConfig(params=param_server,
                     source_pos=[3, -30],
                     sink_pos=[-30, 3],
                     behavior_model=BehaviorIntersectionRuleBased(param_server),
                     controlled_ids=True,
                     min_vel=5.,
                     max_vel=5.,
                     ds_min=5.,
                     ds_max=10.,
                     s_min=15.,
                     s_max=30.))

scenarios = \
  ConfigWithEase(num_scenarios=3,
                 map_file_name=Data.xodr_data("threeway_intersection"),
                 random_seed=0,
                 params=param_server,
                 lane_corridor_configs=lane_corridors)

# viewer
viewer = MPViewer(params=param_server, use_world_bounds=True)

# World Simulation
sim_step_time = param_server["simulation"]["step_time",
                                           "Step-time used in simulation",
                                           0.2]
sim_real_time_factor = param_server["simulation"]["real_time_factor",
                                                  "execution in real-time or faster",
                                                  1.]
# viewer = VideoRenderer(renderer=viewer,
#                        world_step_time=sim_step_time,
#                        fig_path="/Users/hart/2019/bark/video")
env = Runtime(step_time=0.2,
              viewer=viewer,
              scenario_generator=scenarios,
              render=True)

# run 3 scenarios
for _ in range(0, 3):
  env.reset()
  # step each scenario 20 times
  for step in range(0, 80):
    env.step()
    time.sleep(sim_step_time/sim_real_time_factor)

# viewer.export_video(filename="/Users/hart/2019/bark/video/intersection", remove_image_dir=True)
