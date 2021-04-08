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
from bark.runtime.runtime import Runtime
from bark.core.models.behavior import *
from bark.examples.paths import Data


# parameters
param_server = ParameterServer()
param_server["BehaviorLaneChangeRuleBased"]["MinVehicleRearDistance"] = 4.
param_server["BehaviorLaneChangeRuleBased"]["MinVehicleFrontDistance"] = 2.
param_server["BehaviorLaneChangeRuleBased"]["TimeKeepingGap"] = 0.

# custom lane configuration that sets a different behavior model
# and sets the desired speed for the behavior
class HighwayLaneCorridorConfig(LaneCorridorConfig):
  def __init__(self, params=None, **kwargs):
    super().__init__(params=params, **kwargs)
    self._count = 0
  
  def velocity(self):
    return np.random.uniform(2., 10.)

  def behavior_model(self, world):
    # agent_params = ParameterServer()
    params = self._params.AddChild("BehaviorLaneChangeRuleBased"+str(self._count))
    params["BehaviorIDMClassic"]["DesiredVelocity"] = \
      np.random.uniform(5., 10.)
    behavior_model = BehaviorLaneChangeRuleBased(params)
    self._count += 1
    return behavior_model


# configure both lanes of the highway. the right lane has one controlled agent
left_lane = HighwayLaneCorridorConfig(params=param_server,
                                      road_ids=[16],
                                      lane_corridor_id=0)
right_lane = HighwayLaneCorridorConfig(params=param_server,
                                       road_ids=[16],
                                       lane_corridor_id=1,
                                       controlled_ids=True)

# create 5 scenarios
scenarios = \
  ConfigWithEase(
    num_scenarios=5,
    map_file_name=Data.xodr_data("city_highway_straight"),
    random_seed=0,
    params=param_server,
    lane_corridor_configs=[left_lane, right_lane])

# viewer
viewer = MPViewer(params=param_server,
                  x_range=[-75, 75],
                  y_range=[-75, 75],
                  follow_agent_id=True)

sim_step_time = param_server["simulation"]["step_time",
                                          "Step-time used in simulation",
                                          0.05]
sim_real_time_factor = param_server["simulation"][
  "real_time_factor",
  "execution in real-time or faster",
  0.5]

# viewer = VideoRenderer(renderer=viewer,
#                        world_step_time=sim_step_time,
#                        fig_path="/home/hart/Dokumente/2020/bark/video")

# gym like interface
env = Runtime(step_time=0.2,
              viewer=viewer,
              scenario_generator=scenarios,
              render=True)
      


# run 3 scenarios
for episode in range(0, 1):
  env.reset()
  # step each scenario 20 times
  for step in range(0, 70):
    env.step()
    time.sleep(sim_step_time/sim_real_time_factor)

# viewer.export_video(filename="/home/hart/Dokumente/2020/bark/video/video", remove_image_dir=False)
