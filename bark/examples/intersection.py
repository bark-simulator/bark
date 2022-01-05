# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import time
from bark.runtime.commons.parameters import ParameterServer
from bark.runtime.viewer.matplotlib_viewer import MPViewer
from bark.runtime.viewer.video_renderer import VideoRenderer
from bark.runtime.scenario.scenario_generation.config_with_ease import \
  LaneCorridorConfig, ConfigWithEase
from bark.core.models.behavior import *
from bark.runtime.runtime import Runtime
from bark.examples.paths import Data


# Parameters Definitions
param_server_give_way = ParameterServer()
param_server_take_way = ParameterServer()

param_server_give_way["BehaviorIDMLaneTracking"]["CrosstrackErrorGain"] = 2.5
param_server_give_way["BehaviorIDMClassic"]["DesiredVelocity"] = 5.
param_server_give_way["BehaviorIDMClassic"]["MaxLatDifferenceToBeFront"] = 15.0
param_server_give_way["BehaviorIDMClassic"]["MaxAngleDifferenceToBeFront"] = 3.1
param_server_give_way["BehaviorIDMClassic"]["MaxLonDifferenceToBeFront"] = 4.0
param_server_give_way["BehaviorIDMClassic"]["MinimumSpacing"] = 8.0

param_server_take_way["BehaviorIDMLaneTracking"]["CrosstrackErrorGain"] = 2.5
param_server_take_way["BehaviorIDMClassic"]["DesiredVelocity"] = 5.
param_server_take_way["BehaviorIDMClassic"]["MaxLatDifferenceToBeFront"] = 0.0
param_server_take_way["BehaviorIDMClassic"]["MaxAngleDifferenceToBeFront"] = 3.1
param_server_take_way["BehaviorIDMClassic"]["MaxLonDifferenceToBeFront"] = 0.0

# configure the lane corridors and how the agents 
lane_corridors = []
lane_corridors.append(
  LaneCorridorConfig(params=param_server_give_way,
                     source_pos=[-30, -3],
                     sink_pos=[30, -3],
                     behavior_model=BehaviorIDMClassic(param_server_give_way),
                     min_vel=5.,
                     max_vel=5.,
                     ds_min=8.,
                     ds_max=10.,
                     s_min=15.,
                     s_max=35.))
lane_corridors.append(
  LaneCorridorConfig(params=param_server_give_way,
                     source_pos=[30, 3],
                     sink_pos=[-30, 3],
                     behavior_model=BehaviorIDMClassic(param_server_give_way),
                     min_vel=5.,
                     max_vel=5.,
                     ds_min=8.,
                     ds_max=10.,
                     s_min=15.,
                     s_max=35.))
lane_corridors.append(
  LaneCorridorConfig(params=param_server_take_way,
                     source_pos=[3, -30],
                     sink_pos=[-30, 3],
                     behavior_model=BehaviorIDMClassic(param_server_take_way),
                     controlled_ids=True,
                     min_vel=5.,
                     max_vel=5.,
                     ds_min=10.,
                     ds_max=12.,
                     s_min=25.,
                     s_max=30.))

scenarios = \
  ConfigWithEase(num_scenarios=3,
                 map_file_name=Data.xodr_data("threeway_intersection"),
                 random_seed=0,
                 params=param_server_give_way,
                 lane_corridor_configs=lane_corridors)

# viewer
viewer = MPViewer(params=param_server_give_way, use_world_bounds=True)

# World Simulation
sim_step_time = param_server_give_way["simulation"]["step_time",
                                           "Step-time used in simulation",
                                           0.2]
sim_real_time_factor = param_server_give_way["simulation"]["real_time_factor",
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
