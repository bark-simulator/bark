# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import os

from load.benchmark_database import BenchmarkDatabase
from bark.benchmark.benchmark_runner import BenchmarkRunner
from bark.core.world.evaluation import EvaluatorGoalReached, \
  EvaluatorCollisionEgoAgent, EvaluatorStepCount
from bark.runtime.commons.parameters import ParameterServer

behavior_used = None
try:
  from bark.core.models.behavior import BehaviorUCTSingleAgent
  behavior_used = BehaviorUCTSingleAgent
except:
  print("BehaviorUCTSingleAgent not available, rerun example with `bazel run //examples:benchmark_database --define planner_uct=true")
  exit()

db = BenchmarkDatabase(database_root="external/benchmark_database_release")
evaluators = {"success" : EvaluatorGoalReached,
              "collision" : EvaluatorCollisionEgoAgent,
              "max_steps": EvaluatorStepCount}

terminal_when = {"collision" :lambda x: x,
                 "max_steps": lambda x : x>2}

scenario_param_file ="uct_planner.json" # must be within examples params folder
params = ParameterServer(filename= os.path.join("examples/params/", scenario_param_file))
behaviors_tested = {"search5s": behavior_used(params)}

benchmark_runner = BenchmarkRunner(benchmark_database=db,
                                    evaluators=evaluators,
                                    terminal_when=terminal_when,
                                    behaviors=behaviors_tested)

benchmark_runner.run(1)
benchmark_runner.dataframe.to_pickle("uct_planner_results.pickle")
