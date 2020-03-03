# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import unittest
import os
import ray

from load.benchmark_database import BenchmarkDatabase
from serialization.database_serializer import DatabaseSerializer
from modules.benchmark.benchmark_runner import BenchmarkRunner, BenchmarkConfig, BenchmarkResult
from modules.benchmark.benchmark_runner_mp import BenchmarkRunnerMP
from modules.benchmark.benchmark_analyzer import BenchmarkAnalyzer

from modules.runtime.viewer.matplotlib_viewer import MPViewer

from bark.world.evaluation import *
from modules.runtime.commons.parameters import ParameterServer
from bark.models.behavior import BehaviorIDMClassic, BehaviorConstantVelocity

# to find database files
os.chdir("../benchmark_database/")

class DatabaseRunnerTests(unittest.TestCase):
    def test_database_run_and_analyze(self):
        dbs = DatabaseSerializer(test_scenarios=4, test_world_steps=5, num_serialize_scenarios=10)
        dbs.process("database")
        local_release_filename = dbs.release(version="test")

        db = BenchmarkDatabase(database_root=local_release_filename)
        evaluators = {"success" : "EvaluatorGoalReached", "collision" : "EvaluatorCollisionEgoAgent",
                      "max_steps": "EvaluatorStepCount"}
        terminal_when = {"collision" :lambda x: x, "max_steps": lambda x : x>5}
        params = ParameterServer() # only for evaluated agents not passed to scenario!
        behaviors_tested = {"IDM": BehaviorIDMClassic(params), "Const" : BehaviorConstantVelocity(params)}

        benchmark_runner = BenchmarkRunnerMP(benchmark_database=db,
                                           evaluators=evaluators,
                                           terminal_when=terminal_when,
                                           behaviors=behaviors_tested,
                                           log_eval_avg_every=10)

        result = benchmark_runner.run(maintain_history=True)

        result.dump(os.path.join("./benchmark_results.pickle"))
        result_loaded = BenchmarkResult.load(os.path.join("./benchmark_results.pickle"))

        params2 = ParameterServer()
        viewer = MPViewer(
              params=params2,
              use_world_bounds=True)
        analyzer = BenchmarkAnalyzer(benchmark_result=result_loaded)
        viewer.show(block=False)

        analyzer.visualize(criteria={"behavior": lambda x: x=="IDM", "success": lambda x : not x}, \
                          viewer = viewer, real_time_factor=10)


if __name__ == '__main__':
    unittest.main()


