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
from modules.runtime.viewer.panda3d_easy import Panda3dViewer

from bark.world.opendrive import *
from bark.world.goal_definition import *

# parameters
param_server = ParameterServer()

# scenario
class CustomLaneCorridorConfig(LaneCorridorConfig):
  def __init__(self,
               road_ids=[16],
               lane_corridor_id=0,
               params=None,
               controlled_agent=None):
    super(CustomLaneCorridorConfig, self).__init__(road_ids,
                                                   lane_corridor_id,
                                                   params)
    self._controlled_agent = controlled_agent

  def position(self, world, min_s=10., max_s=50.):
    """Min. and max values where the agents should be places
    """
    return super(CustomLaneCorridorConfig, self).position(world, min_s, max_s)

  def ds(self, s_min=20., s_max=35.):
    """Sample distance on the route
    """
    return np.random.uniform(s_min, s_max)

  def controlled_ids(self, agent_list):
    """Define controlled agents
    """
    if self._controlled_agent is None:
      return []
    else:
      return super().controlled_ids(agent_list)
  
  def controlled_goal(self, world):
    if self._controlled_agent is not None:
      road_corr = world.map.GetRoadCorridor(self._road_ids, XodrDrivingDirection.forward)
      lane_corr = road_corr.lane_corridors[0]
      return GoalDefinitionPolygon(road_corr.lane_corridors[0].polygon)
    else:
      super().controlled_goal(world)


# configure both lanes of the highway. the right lane has one controlled agent
left_lane = CustomLaneCorridorConfig(lane_corridor_id=0,
                                     params=param_server,
                                     road_ids=[0])
right_lane = CustomLaneCorridorConfig(lane_corridor_id=1,
                                      params=param_server,
                                      controlled_agent=True,
                                      road_ids=[0, 1])


# create 5 scenarios
scenarios = \
  ConfigWithEase(num_scenarios=5,
                 map_file_name="modules/runtime/tests/data/DR_DEU_Merging_MT_v01_shifted.xodr",
                 random_seed=0,
                 params=param_server,
                 lane_corridor_configs=[left_lane, right_lane])

# viewer
viewer = MPViewer(params=param_server,
                  x_range=[-40, 40],
                  y_range=[-40, 40],
                  follow_agent_id=True)
viewer = Panda3dViewer(params=param_server,
                       x_range=[-40, 40],
                       y_range=[-40, 40],
                       follow_agent_id=True,
                       light_pose=[1000, 1000, 100000],
                       camera_pose=[1000, 980, 100])
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
env.reset()
# step each scenario 20 times
for step in range(0, 10):
  env.step()
  time.sleep(sim_step_time/sim_real_time_factor)
