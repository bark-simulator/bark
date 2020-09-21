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

from bark.core.world.opendrive import *
from bark.core.world.goal_definition import *
from bark.core.models.behavior import *
from bark.core.commons import SetVerboseLevel

try:
    from bark.core.world.evaluation import EvaluatorRss
except:
    raise ImportError(
        "This example requires building RSS, please run with \"bazel run //examples:merging_rss --define rss=true\"")

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

param_server["Visualization"]["Evaluation"]["DrawRssDebugInfo"] = True
param_server["Visualization"]["Evaluation"]["DrawRssSafetyResponses"] = True

SetVerboseLevel(0)

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

map_path = "bark/runtime/tests/data/DR_DEU_Merging_MT_v01_centered.xodr"

scenarios = \
    ConfigWithEase(num_scenarios=3,
                   map_file_name=map_path,
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

# Defining vehicles dynamics for RSS

# Input format:
# [longitudinal max acceleration, longitudinal max braking, longitudinal min acceleration,
# longitudinal min brake correct, lateral max acceleration, lateral min braking,
# lateral flucatuation_margin, agent response time]
#
# Detailed explanation please see:
# https://intel.github.io/ad-rss-lib/ad_rss/Appendix-ParameterDiscussion/#parameter-discussion

# Default dynamics for every agent if it is not defined indivually
default_vehicle_dynamics = [1.7, -1.7, -1.69, -1.67, 0.2, -0.8, 0.1, 1.]

# Indivually dynamics, each defined with the agent id
agents_vehicle_dynamics = {1: [1.7, -1.7, -1.69, -1.67, 0.2, -0.8, 0.1, 1.],
                           2: [1.71, -1.7, -1.69, -1.67, 0.2, -0.8, 0.1, 1.]}

# Example of using RSS to evaluate the safety situation of the evaluating agent.
# The evaluating agent is defined with agent_id when initializing EvaluatorRss.


def print_rss_safety_response(evaluator_rss, world):
    # Evaluating with RSS is quite computionally expensive
    print("Overall safety response: ", evaluator_rss.Evaluate(world))
    # print("Pairwise safety response: ",
    #       evaluator_rss.PairwiseEvaluate(world))
    # print("Pairwise directional safety response: ",
    #       evaluator_rss.PairwiseDirectionalEvaluate(world))


param_server["EvalutaorRss"]["MapFilename"] = map_path
param_server["EvalutaorRss"]["DefaultVehicleDynamics"] = default_vehicle_dynamics
param_server["EvalutaorRss"]["SpecificAgentVehicleDynamics"] = agents_vehicle_dynamics
param_server["EvalutaorRss"]["CheckingRelevantRange"] = 1


# run 3 scenarios
for episode in range(0, 100):
    env.reset()
    current_world = env._world
    eval_agent_id = env._scenario._eval_agent_ids[0]

    # There are two ways to upset EvaluatorRss
    # evaluator_rss = EvaluatorRss(eval_agent_id, map_path,
    #                              default_vehicle_dynamics,
    #                              agents_vehicle_dynamics,
    #                              checking_relevent_range=1)
    evaluator_rss = EvaluatorRss(eval_agent_id, param_server)

    current_world.AddEvaluator("rss", evaluator_rss)

    # step each scenario 40 times
    for step in range(0, 40):
        env.step()
        print_rss_safety_response(evaluator_rss, current_world)
        time.sleep(sim_step_time / sim_real_time_factor)

# viewer.export_video(filename="/home/hart/Dokumente/2020/bark/video/video", remove_image_dir=False)
