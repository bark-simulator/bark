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
from bark.examples.paths import Data

from bark.core.world.opendrive import *
from bark.core.world.goal_definition import *
from bark.core.models.behavior import *
from bark.core.models.observer import *
from bark.core.commons import SetVerboseLevel

# parameters
param_server = ParameterServer()

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

param_server["BehaviorIDMClassic"]["BrakeForLaneEnd"] = True
param_server["BehaviorIDMClassic"]["BrakeForLaneEndEnabledDistance"] = 60.0
param_server["BehaviorIDMClassic"]["BrakeForLaneEndDistanceOffset"] = 30.0
param_server["BehaviorLaneChangeRuleBased"]["MinRemainingLaneCorridorDistance"] = 80.
param_server["BehaviorLaneChangeRuleBased"]["MinVehicleRearDistance"] = 0.
param_server["BehaviorLaneChangeRuleBased"]["MinVehicleFrontDistance"] = 0.
param_server["BehaviorLaneChangeRuleBased"]["TimeKeepingGap"] = 0.
param_server["BehaviorMobilRuleBased"]["Politeness"] = 0.0
param_server["BehaviorIDMClassic"]["DesiredVelocity"] = 10.
param_server["World"]["FracLateralOffset"] = 0.8
param_server["Visualization"]["Agents"]["DrawEvalGoals"] = False

SetVerboseLevel(0)

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

# configure parametric observer,
# multivariate distributions modeling random state deviations 
# in directions: long. , lat
params_parametric = ParameterServer()
params_parametric["ObserverModelParametric"] \
      ["EgoStateDeviationDist"]["Covariance"] = [[0.01, 0.0, 0.0, 0.0],  #  4 x 4 elements
                                                 [0.0, 0.01, 0.0, 0.0],
                                                 [0.0, 0.00, 0.00001, 0.0],
                                                 [0.0, 0.00, 0.0, 0.0001]]
params_parametric["ObserverModelParametric"] \
      ["EgoStateDeviationDist"]["Mean"] =       [0.0, 0.0, 0.0, 0.0] # 4 elements
params_parametric["ObserverModelParametric"] \
      ["OtherStateDeviationDist"]["Covariance"] = [[0.05, 0.0, 0.0, 0.0],  #  4 x 4 elements
                                                 [0.0, 0.01, 0.0, 0.0],
                                                 [0.0, 0.00, 0.001, 0.0],
                                                 [0.0, 0.00, 0.0, 0.05]]
params_parametric["ObserverModelParametric"] \
      ["OtherStateDeviationDist"]["Mean"] =       [0.0, 0.0, 0.0, 0.0] # 4 elements
parametric_observer = ObserverModelParametric(params_parametric)


scenarios = \
  ConfigWithEase(num_scenarios=3,
                 map_file_name=Data.xodr_data("DR_DEU_Merging_MT_v01_shifted"),
                 random_seed=0,
                 params=param_server,
                 lane_corridor_configs=[left_lane, right_lane],
                 observer_model=parametric_observer)

# viewer
viewer = MPViewer(params=param_server,
                  x_range=[-35, 35],
                  y_range=[-35, 35],
                  follow_agent_id=True)

sim_step_time = param_server["simulation"]["step_time",
                                           "Step-time used in simulation",
                                           0.2]
sim_real_time_factor = param_server["simulation"]["real_time_factor",
                                                  "execution in real-time or faster",
                                                  1.]

viewer = VideoRenderer(renderer=viewer,
                       world_step_time=sim_step_time)

env = Runtime(step_time=0.2,
              viewer=viewer,
              scenario_generator=scenarios,
              render=True,
              maintain_world_history=True)

# run 3 scenarios
for _ in range(0, 3):
  env.reset()
  # step each scenario 20 times
  for step in range(0, 10):
    env.step()
    time.sleep(sim_step_time/sim_real_time_factor)
    
  df = env.ExtractTimeSeries()
  print(df)

viewer.export_video(filename="./example_video", remove_image_dir=False)
