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

        fig = plt.figure(figsize=[10, 10])
        viewer = MPViewer(
              params=params2,
              center=[5112, 5165],
              y_length = 120,
              enforce_y_length=True,
              axis = fig.gca())

        analyzer = BenchmarkAnalyzer(benchmark_result=result_loaded)
        configs = analyzer.find_configs(criteria={"behavior": lambda x: x=="IDM", "success": lambda x : not x})
        configs_const = analyzer.find_configs(criteria={"behavior": lambda x: x=="Const", "success": lambda x : not x})

        #analyzer.visualize(configs_idx_list = configs,
                         # viewer = viewer, real_time_factor=10, fontsize=12)
        plt.close(fig)

        
        fig, (ax1, ax2) = plt.subplots(1, 2)
        viewer1 = MPViewer(
              params=params2,
              center=[5112, 5165],
              y_length = 120,
              enforce_y_length=True,
              axis = ax1)
        viewer2 = MPViewer(
              params=params2,
              center=[5112, 5165],
              y_length = 120,
              enforce_y_length=True,
              axis = ax2)
        analyzer.visualize(configs_idx_list = [configs[1:3], configs_const[1:3]],
                          viewer = [viewer1, viewer2], viewer_names=["IDM", "ConstVelocity"], real_time_factor=1, fontsize=12)



if __name__ == '__main__':
    unittest.main()


