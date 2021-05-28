# Copyright (c) 2021 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.


# NOTE: To run this example, you need to clone BARKSCAPE
#       Clone using: git clone https://github.com/bark-simulator/barkscape
#       Run the web-inferfce: bazel run //barkscape/web:run

import sys, os, logging
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from bark.runtime.commons.parameters import ParameterServer
from bark.runtime.viewer.buffered_viewer import BufferedViewer
from bark.runtime.scenario.scenario_generation.config_with_ease import \
  LaneCorridorConfig, ConfigWithEase
from bark.runtime.runtime import Runtime
from bark.examples.paths import Data

from bark.core.world.opendrive import *
from bark.core.world.goal_definition import *
from bark.core.models.behavior import *
from bark.core.commons import SetVerboseLevel

# BARKSCAPE
from barkscape.server.base_server import BaseServer
from barkscape.server.runners.bark_runner import BARKRunner

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


if __name__ == "__main__":
  # configure lanes
  param_server = ParameterServer()
  # NOTE: merging
  # left_lane = CustomLaneCorridorConfig(params=param_server,
  #                                     lane_corridor_id=0,
  #                                     road_ids=[0, 1],
  #                                     behavior_model=BehaviorMobilRuleBased(param_server),
  #                                     s_min=5.,
  #                                     s_max=50.)
  # right_lane = CustomLaneCorridorConfig(params=param_server,
  #                                       lane_corridor_id=1,
  #                                       road_ids=[0, 1],
  #                                       controlled_ids=True,
  #                                       behavior_model=BehaviorMobilRuleBased(param_server),
  #                                       s_min=5.,
  #                                       s_max=20.)
  # scenarios = \
  #   ConfigWithEase(num_scenarios=3,
  #                 map_file_name=Data.xodr_data("DR_DEU_Merging_MT_v01_shifted"),
  #                 random_seed=0,
  #                 params=param_server,
  #                 lane_corridor_configs=[left_lane, right_lane])
    
  # NOTE: intersection
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
  
  viewer = BufferedViewer()
  env = Runtime(step_time=0.2,
                viewer=viewer,
                scenario_generator=scenarios,
                render=True,
                maintain_world_history=True)

  # run BARKSCAPE
  logger = logging.getLogger()
  bark_server = BaseServer(
    runner=BARKRunner, runnable_object=env, logger=logger)
  bark_server.Start()
