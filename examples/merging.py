# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import numpy as np
import time
from modules.runtime.commons.parameters import ParameterServer
from modules.runtime.viewer.matplotlib_viewer import MPViewer
from modules.runtime.viewer.video_renderer import VideoRenderer
from modules.runtime.scenario.scenario_generation.config_with_ease import \
  LaneCorridorConfig, ConfigWithEase
from modules.runtime.runtime import Runtime
from modules.runtime.viewer.panda3d_easy import Panda3dViewer

from bark.world.opendrive import *
from bark.world.goal_definition import *
from bark.models.behavior import *

# parameters
param_server = ParameterServer()

# scenario
class CustomLaneCorridorConfig(LaneCorridorConfig):
  def __init__(self,
               road_ids=[16],
               lane_corridor_id=0,
               params=None,
               controlled_agent=None,
               min_s=0,
               max_s=50):
    super(CustomLaneCorridorConfig, self).__init__(road_ids,
                                                   lane_corridor_id,
                                                   params)
    self._controlled_agent = controlled_agent
    self._min_s = min_s
    self._max_s = max_s

  def position(self, world, min_s=0., max_s=25.):
    """Min. and max values where the agents should be places
    """
    return super(CustomLaneCorridorConfig, self).position(
      world, self._min_s, self._max_s)

  def ds(self, s_min=7.5, s_max=15.):
    """Sample distance on the route
    """
    return np.random.uniform(s_min, s_max)

  def velocity(self, min_vel=8., max_vel=10.):
    return np.random.uniform(low=min_vel, high=max_vel)

  def controlled_ids(self, agent_list):
    """Define controlled agents
    """
    if self._controlled_agent is None:
      return []
    else:
      return super().controlled_ids(agent_list)
  
  def goal(self, world):
    road_corr = world.map.GetRoadCorridor(
      self._road_ids, XodrDrivingDirection.forward)
    lane_corr = road_corr.lane_corridors[0]
    return GoalDefinitionPolygon(road_corr.lane_corridors[0].polygon)
  
  def behavior_model(self, world):
    return BehaviorSimpleRuleBased(self._params)
    # return BehaviorIntersectionRuleBased(self._params)

    
# configure both lanes of the highway. the right lane has one controlled agent
left_lane = CustomLaneCorridorConfig(lane_corridor_id=0,
                                     params=param_server,
                                     road_ids=[0, 1],
                                     min_s=0.,
                                     max_s=50.)
right_lane = CustomLaneCorridorConfig(lane_corridor_id=1,
                                      params=param_server,
                                      controlled_agent=True,
                                      road_ids=[0, 1],
                                      min_s=0.,
                                      max_s=20.)


# create 5 scenarios
# BehaviorSimpleRuleBased::MinVehicleRearDistance
param_server["BehaviorIDMClassic"]["BrakeForLaneEnd"] = True
param_server["BehaviorSimpleRuleBased"]["MinRemainingLaneCorridorDistance"] = 70.
param_server["BehaviorSimpleRuleBased"]["MinVehicleRearDistance"] = 2.5
param_server["BehaviorSimpleRuleBased"]["MinVehicleFrontDistance"] = 2.5

scenarios = \
  ConfigWithEase(num_scenarios=3,
                 map_file_name="modules/runtime/tests/data/DR_DEU_Merging_MT_v01_shifted.xodr",
                 random_seed=0,
                 params=param_server,
                 lane_corridor_configs=[left_lane, right_lane])

# viewer
viewer = MPViewer(params=param_server,
                  x_range=[-35, 35],
                  y_range=[-35, 35],
                  follow_agent_id=True)
# viewer = Panda3dViewer(params=param_server,
#                        x_range=[-40, 40],
#                        y_range=[-40, 40],
#                        follow_agent_id=True,
#                        light_pose=[1000, 1000, 100000],
#                        camera_pose=[1000, 980, 100])
# gym like interface

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
  for step in range(0, 60):
    env.step()
    time.sleep(sim_step_time/sim_real_time_factor)

# viewer.export_video(filename="/Users/hart/2019/bark/video/video", remove_image_dir=False)
