# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import unittest
import os
import ray

import matplotlib.pyplot as plt

from load.benchmark_database import BenchmarkDatabase
from serialization.database_serializer import DatabaseSerializer
from bark.benchmark.benchmark_runner import BenchmarkRunner, BenchmarkConfig, BenchmarkResult
from bark.benchmark.benchmark_runner_mp import BenchmarkRunnerMP
from bark.benchmark.benchmark_analyzer import BenchmarkAnalyzer

from bark.runtime.viewer.matplotlib_viewer import MPViewer
from bark.runtime.viewer.video_renderer import VideoRenderer

from bark.core.world.evaluation import *
from bark.runtime.commons.parameters import ParameterServer
from bark.core.models.behavior import BehaviorIDMClassic, BehaviorConstantVelocity, BehaviorUCTSingleAgentMacroActions
from bark.runtime.viewer.video_renderer import VideoRenderer

dbs = DatabaseSerializer(test_scenarios=2, test_world_steps=2, num_serialize_scenarios=100) # increase the number of serialize scenarios to 100
dbs.process("examples/mcts_rl/database")
local_release_filename = dbs.release(version="test")#test

db = BenchmarkDatabase(database_root=local_release_filename)

evaluators = {"success" : "EvaluatorGoalReached", "collision" : "EvaluatorCollisionEgoAgent",
            "max_steps": "EvaluatorStepCount"}
terminal_when = {"collision" :lambda x: x, "max_steps": lambda x : x>40, "success" : lambda x: x}


params = ParameterServer()
params["BehaviorUctSingleAgent"]["UseRandomHeuristic"] = False
params["BehaviorUctSingleAgent"]["Mcts"]["UctStatistic"]["ReturnLowerBound"] = -10000.0
params["BehaviorUctSingleAgent"]["Mcts"]["UctStatistic"]["ReturnUpperBound"] = 10000.0
default_model = BehaviorUCTSingleAgentMacroActions(params)
params.Save(filename="./macro_action_params.json")

scenario_param_file ="macro_action_params.json" # must be within examples params folder
params1 = ParameterServer(filename= os.path.join("examples/mcts_rl/params/",scenario_param_file))
params2 = ParameterServer(filename= os.path.join("examples/mcts_rl/params/",scenario_param_file))
params3 = ParameterServer(filename= os.path.join("examples/mcts_rl/params/",scenario_param_file))
# Params NN Heuristic
params3["BehaviorUctSingleAgent"]["UseRandomHeuristic"]=False
params3["BehaviorUctSingleAgent"]["UseNNHeuristic"]=True
params3["BehaviorUctSingleAgent"]["Savedmodeldirectory"]="/home/guoyujian/model2_cdqn_100k/model"
params3["BehaviorUctSingleAgent"]["Mcts"]["UctStatistic"]["ReturnLowerBound"] = -10000.0
params3["BehaviorUctSingleAgent"]["Mcts"]["UctStatistic"]["ReturnUpperBound"] =10000.0
# Params Domain Heuristic
params2["BehaviorUctSingleAgent"]["UseRandomHeuristic"]=False
params2["BehaviorUctSingleAgent"]["UseNNHeuristic"]=False
params2["BehaviorUctSingleAgent"]["Mcts"]["UctStatistic"]["ReturnLowerBound"] = -10000.0
params2["BehaviorUctSingleAgent"]["Mcts"]["UctStatistic"]["ReturnUpperBound"] = 10000.0
# Params Random Heuristic
params1["BehaviorUctSingleAgent"]["UseRandomHeuristic"]=True
params1["BehaviorUctSingleAgent"]["UseNNHeuristic"]=False
params1["BehaviorUctSingleAgent"]["Mcts"]["UctStatistic"]["ReturnLowerBound"] = -10000.0
params1["BehaviorUctSingleAgent"]["Mcts"]["UctStatistic"]["ReturnUpperBound"] = 10000.0
behaviors_tested = {"RandomHeuristic": BehaviorUCTSingleAgentMacroActions(params1),"DOMAINHeuristic": BehaviorUCTSingleAgentMacroActions(params2),"NNHeuristic": BehaviorUCTSingleAgentMacroActions(params3)}

benchmark_runner = BenchmarkRunner(benchmark_database=db,
                                  evaluators=evaluators,
                                  terminal_when=terminal_when,
                                  behaviors=behaviors_tested,
                                  log_eval_avg_every=10)#log_eval_avg_every=10

viewer1 = MPViewer(
                  x_range=[-75, 75],
                  y_range=[-75, 75],
                  follow_agent_id=True)
video_renderer = VideoRenderer(renderer=viewer1, world_step_time=0.2)


result = benchmark_runner.run(maintain_history=True, viewer=viewer1)

video_renderer.export_video(filename="./heuristic_test_video.mp4", remove_image_dir=True)
result.dump(os.path.join("./benchmark_results.pickle"))


# Put this code in another file and run it after the benchmark based on the saved data
result_loaded = BenchmarkResult.load(os.path.join("./benchmark_results.pickle"))

data_frame = result_loaded.get_data_frame()
data_frame["max_steps"] = data_frame.Terminal.apply(lambda x: "max_steps" in x and (not "collision" in x))
data_frame["success"] = data_frame.Terminal.apply(lambda x: "success" in x and (not "collision" in x) and (not "max_steps" in x))

data_frame.fillna(-1)
dfg = data_frame.fillna(-1).groupby(["behavior"]).mean()
print(dfg.to_string())
