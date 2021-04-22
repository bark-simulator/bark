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
from bark.core.commons import SetVerboseLevel
from bark.core.geometry import *

try:
    from bark.core.world.evaluation import EvaluatorRSS
except:
    raise ImportError(
        "This example requires building RSS, please run with \"bazel run //examples:merging_rss --define rss=true\"")

# parameters
param_server = ParameterServer()

class CustomLaneCorridorConfig(LaneCorridorConfig):
    def __init__(self,
                 params=None,
                 **kwargs):
        super(CustomLaneCorridorConfig, self).__init__(params, **kwargs)

    def goal(self, world):
        road_corr = world.map.GetRoadCorridor(
            self._road_ids, XodrDrivingDirection.forward)
        lane_corr = self._road_corridor.lane_corridors[0]
        goal_polygon = Polygon2d([0, 0, 0], [Point2d(-1, -1), Point2d(-1, 1), Point2d(1, 1), Point2d(1, -1)])
        goal_polygon = goal_polygon.Translate(Point2d(lane_corr.center_line.ToArray()[-1, 0], lane_corr.center_line.ToArray()[-1, 1]))
        return GoalDefinitionPolygon(goal_polygon)


param_server["BehaviorIDMClassic"]["BrakeForLaneEnd"] = True
param_server["BehaviorIDMClassic"]["BrakeForLaneEndEnabledDistance"] = 25.0
param_server["BehaviorIDMClassic"]["BrakeForLaneEndDistanceOffset"] = 25.0
param_server["BehaviorLaneChangeRuleBased"]["MinRemainingLaneCorridorDistance"] = 20.
param_server["BehaviorLaneChangeRuleBased"]["MinVehicleRearDistance"] = 0.
param_server["BehaviorLaneChangeRuleBased"]["MinVehicleFrontDistance"] = 0.
param_server["BehaviorLaneChangeRuleBased"]["TimeKeepingGap"] = 0.
param_server["BehaviorMobilRuleBased"]["Politeness"] = 0.0
param_server["BehaviorIDMClassic"]["DesiredVelocity"] = 10.
param_server["World"]["LateralDifferenceThreshold"] = 2.0
param_server["Visualization"]["Agents"]["DrawAgentId"] =  True

# param_server["Visualization"]["Evaluation"]["DrawRssDebugInfo"] = True
# param_server["Visualization"]["Evaluation"]["DrawRssSafetyResponses"] = True
param_server["Visualization"]["Agents"]["DrawEvalGoals"] = False
param_server["Visualization"]["Evaluation"]["DrawEgoRSSSafetyResponses"] = True

SetVerboseLevel(4)

# configure both lanes of the highway. the right lane has one controlled agent
left_lane = CustomLaneCorridorConfig(params=param_server,
                                     lane_corridor_id=0,
                                     road_ids=[0, 1],
                                     behavior_model=BehaviorMobilRuleBased(
                                         param_server),
                                     s_min=5.,
                                     s_max=50.)
right_lane = CustomLaneCorridorConfig(params=param_server,
                                      lane_corridor_id=1,
                                      road_ids=[0, 1],
                                      controlled_ids=True,
                                      behavior_model=BehaviorMobilRuleBased(
                                          param_server),
                                      s_min=5.,
                                      s_max=20.)

scenarios = \
    ConfigWithEase(num_scenarios=3,
                   map_file_name=Data.xodr_data("DR_DEU_Merging_MT_v01_centered"),
                   random_seed=0,
                   params=param_server,
                   lane_corridor_configs=[left_lane, right_lane])

# viewer
viewer = MPViewer(params=param_server,
                  # x_range=[-35, 35],
                  # y_range=[-35, 35],
                  follow_agent_id=False)

sim_step_time = param_server["simulation"]["step_time",
                                           "Step-time used in simulation",
                                           0.05]
sim_real_time_factor = param_server["simulation"]["real_time_factor",
                                                  "execution in real-time or faster",
                                                  1.]

viewer = VideoRenderer(renderer=viewer,
                       world_step_time=sim_step_time,
                       fig_path="/tmp/video")

env = Runtime(step_time=0.2,
              viewer=viewer,
              scenario_generator=scenarios,
              render=True,
              maintain_world_history=True)

# Defining vehicles dynamics for RSS

# Input format:
# [longitudinal max acceleration, longitudinal max braking, longitudinal min acceleration,
# longitudinal min brake correct, lateral max acceleration, lateral min braking,
# lateral flucatuation_margin, agent response time]
#
# Detailed explanation please see:
# https://intel.github.io/ad-rss-lib/ad_rss/Appendix-ParameterDiscussion/#parameter-discussion

# Example of using RSS to evaluate the safety situation of the evaluating agent.
# The evaluating agent is defined with agent_id when initializing EvaluatorRSS.

def print_rss_safety_response(evaluator_rss, world):
    # Evaluating with RSS is quite computionally expensive
    print("Overall safety response: ", evaluator_rss.Evaluate(world))
    # print("Pairwise safety response: ",
    #       evaluator_rss.PairwiseEvaluate(world))
    # print("Pairwise directional safety response: ",
    #       evaluator_rss.PairwiseDirectionalEvaluate(world))


param_server["EvaluatorRss"]["MapFilename"] = Data.xodr_data("DR_DEU_Merging_MT_v01_centered")

# run n scenarios
for episode in range(0, 10):
  env.reset()
  current_world = env._world
  eval_agent_id = env._scenario._eval_agent_ids[0]
  current_world.agents[eval_agent_id].behavior_model = \
    BehaviorRSSConformant(param_server)
  evaluator_rss = EvaluatorRSS(eval_agent_id, param_server)
  current_world.AddEvaluator("rss", evaluator_rss)

  # step each scenario 40 times
  for step in range(0, 40):
    env.step()
    # print_rss_safety_response(evaluator_rss, current_world)
    time.sleep(sim_step_time / sim_real_time_factor)

  df = env.ExtractTimeSeries()
  print(df)

# viewer.export_video(filename="/tmp/merging_rss", remove_image_dir=False)
