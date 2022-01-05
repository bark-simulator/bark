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
param_server["BehaviorLaneChangeRuleBased"]["MinVehicleRearDistance"] = .1
param_server["BehaviorLaneChangeRuleBased"]["MinVehicleFrontDistance"] = .2
param_server["BehaviorLaneChangeRuleBased"]["TimeKeepingGap"] = 0.

# custom lane configuration that sets a different behavior model
# and sets the desired speed for the behavior
class HighwayLaneCorridorConfigLaneChangers(LaneCorridorConfig):
  def __init__(self, params=None, **kwargs):
    super().__init__(params=params, **kwargs)
    self._count = 0
  
  def velocity(self):
    return np.random.uniform(4., 5.)

  def behavior_model(self, world):
    # agent_params = ParameterServer()
    params = self._params.AddChild("BehaviorLaneChangeRuleBased"+str(self._count))
    params["BehaviorIDMClassic"]["DesiredVelocity"] = \
      np.random.uniform(5., 10.)
    behavior_model = BehaviorLaneChangeRuleBased(params)
    self._count += 1
    return behavior_model

class HighwayLaneCorridorConfigIntentBased(LaneCorridorConfig):
  def __init__(self, params=None, **kwargs):
    super().__init__(params=params, **kwargs)
    self._count = 0
  
  def velocity(self):
    return np.random.uniform(2., 10.)

  def behavior_model(self, world):
    # agent_params = ParameterServer()
    params = self._params.AddChild("BehaviorIDMStochastic"+str(self._count))
    # all other idm parameters do not change over time
    params["BehaviorIDMStochastic::SpacingDistribution::DistributionType"] = "FixedValue"
    params["BehaviorIDMStochastic::SpacingDistribution::FixedValue"] = [0.0]
    params["BehaviorIDMStochastic::MaxAccDistribution::DistributionType"] = "FixedValue"
    params["BehaviorIDMStochastic::MaxAccDistribution::FixedValue"] = [1.8]
    params["BehaviorIDMStochastic::DesiredVelDistribution::DistributionType"] = "FixedValue"
    params["BehaviorIDMStochastic::DesiredVelDistribution::FixedValue"] = [10.0]
    params["BehaviorIDMStochastic::ComftBrakingDistribution::DistributionType"] = "FixedValue"
    params["BehaviorIDMStochastic::ComftBrakingDistribution::FixedValue"] = [1.8]
    params["BehaviorIDMStochastic::CoolnessFactorDistribution::DistributionType"] = "FixedValue"
    params["BehaviorIDMStochastic::CoolnessFactorDistribution::FixedValue"] = [0.0]
    params["BehaviorIDMStochastic::HeadwayDistribution::DistributionType"] = "FixedValue"
    params["BehaviorIDMStochastic::HeadwayDistribution::FixedValue"] = [0.2]

    params["BehaviorIDMStochastic::UseIntentionMechanism"] = True

    # minimum and maximum time yielding intent remains (uniform distribution)
    params["BehaviorIDMStochastic::YieldingDurationDistribution::LowerBound"] = 5.0
    params["BehaviorIDMStochastic::YieldingDurationDistribution::UpperBound"] = 7.0

    # minimum and maximum time no yielding intent remains (uniform distribution)
    params["BehaviorIDMStochastic::NoYieldingDurationDistribution::LowerBound"] = 1.0
    params["BehaviorIDMStochastic::NoYieldingDurationDistribution::UpperBound"] = 2.0

    behavior_model = BehaviorIDMStochastic(params)
    self._count += 1
    return behavior_model


# configure both lanes of the highway. the right lane has one controlled agent
left_lane = HighwayLaneCorridorConfigIntentBased(params=param_server, ds_min = 10, ds_max = 15,
                                      road_ids=[16],
                                      lane_corridor_id=0)
right_lane = HighwayLaneCorridorConfigLaneChangers(params=param_server, ds_min = 20,
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
                  center=[4, -91],
                  y_length = 100,
                  enforce_y_length=True,
                  follow_agent_id=None,
                  use_world_bounds=False)

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
for episode in range(0, 5):
  env.reset()
  # step each scenario 20 times
  for step in range(0, 40):
    env.step()
    time.sleep(sim_step_time/sim_real_time_factor)

# viewer.export_video(filename="/home/hart/Dokumente/2020/bark/video/video", remove_image_dir=False)
