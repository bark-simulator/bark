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
from bark.runtime.viewer.panda3d_easy import Panda3dViewer
from bark.core.models.behavior import *

from bark.core.world.evaluation import EvaluatorRss

# parameters
param_server = ParameterServer(filename="examples/params/centered_highway_merge_configurable.json")
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

  # def controlled_ids(self, agent_list):
  #   """Returns an ID-List of controlled agents
  #   """
  #   if self._controlled_ids is None:
  #     return []
  #   random_int = [agent_list[0]]
  #   return random_int


# configure both lanes of the highway. the right lane has one controlled agent
left_lane = HighwayLaneCorridorConfig(params=param_server,
                                      road_ids=[16],
                                      lane_corridor_id=0)
right_lane = HighwayLaneCorridorConfig(params=param_server,
                                       road_ids=[16],
                                       lane_corridor_id=1,
                                       controlled_ids=True)

map_path="bark/runtime/tests/data/centered_city_highway_straight.xodr"

# create 5 scenarios
scenarios = \
  ConfigWithEase(
    num_scenarios=5,
    map_file_name=map_path,
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
# for episode in range(0, 1):
#   env.reset()
#   # step each scenario 20 times
#   for step in range(0, 70):
#     env.step()
#     time.sleep(sim_step_time/sim_real_time_factor)

# viewer.export_video(filename="/home/hart/Dokumente/2020/bark/video/video", remove_image_dir=False)

for _ in range(0, 5):
    scenario, idx = scenarios.get_next_scenario()
    world = scenario.GetWorldState()
    e = EvaluatorRss(scenario._eval_agent_ids[0], map_path, 
                 [1.7, -1.7, 0., -1.67, 0.2, -0.8, 0.1, 1.])
    for _ in range(0, 70): 
      viewer.drawWorld(world, eval_agent_ids=scenario._eval_agent_ids, scenario_idx=idx )
      viewer.drawSafetyResponses(world,scenario._eval_agent_ids[0],e.PairwiseEvaluate(world))
      world.Step(0.2)
      viewer.clear()