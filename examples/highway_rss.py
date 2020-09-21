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

try:
    from bark.core.world.evaluation import EvaluatorRss
except:
    raise ImportError(
        "This example requires building RSS, please run with \"bazel run //examples:highway_rss --define rss=true\"")

# parameters
param_server = ParameterServer(
    filename="examples/params/highway_centered_merge_configurable.json")
param_server["BehaviorLaneChangeRuleBased"]["MinVehicleRearDistance"] = 4.
param_server["BehaviorLaneChangeRuleBased"]["MinVehicleFrontDistance"] = 2.
param_server["BehaviorLaneChangeRuleBased"]["TimeKeepingGap"] = 0.

param_server["Visualization"]["Evaluation"]["DrawRssDebugInfo"] = True
param_server["Visualization"]["Evaluation"]["DrawRssSafetyResponses"] = True

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
        params = self._params.AddChild(
            "BehaviorLaneChangeRuleBased"+str(self._count))
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

map_path = "bark/runtime/tests/data/city_highway_straight_centered.xodr"

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


def print_rss_safety_response(evaluator_rss, world):
    # Example of using RSS to evaluate the safety situation of the evaluating agent.
    # The evaluating agent is defined with agent_id when initializing EvaluatorRss.
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