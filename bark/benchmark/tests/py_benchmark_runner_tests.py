# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import unittest
import os
import time
import ray

try:
    import debug_settings
except:
    pass

from load.benchmark_database import BenchmarkDatabase
from serialization.database_serializer import DatabaseSerializer
from bark.benchmark.benchmark_runner import BenchmarkRunner, BenchmarkConfig
from bark.benchmark.benchmark_runner_mp import BenchmarkRunnerMP, _BenchmarkRunnerActor, \
  deserialize_benchmark_config, serialize_benchmark_config

from bark.runtime.viewer.matplotlib_viewer import MPViewer

from bark.core.world.evaluation import *
from bark.core.world.evaluation.ltl import ConstantLabelFunction
from bark.runtime.commons.parameters import ParameterServer
from bark.core.models.behavior import BehaviorIDMClassic, BehaviorConstantAcceleration

try: # bazel run
  os.chdir("../benchmark_database/")
except: # debug
  os.chdir("bazel-bin/bark/benchmark/tests/py_benchmark_runner_tests.runfiles/benchmark_database")

class DatabaseRunnerTests(unittest.TestCase):

    def test_database_runner(self):
        dbs = DatabaseSerializer(test_scenarios=4, test_world_steps=5, num_serialize_scenarios=2)
        dbs.process("data/database1")
        local_release_filename = dbs.release(version="test")

        db = BenchmarkDatabase(database_root=local_release_filename)
        evaluators = {"success" : "EvaluatorGoalReached", "collision" : "EvaluatorCollisionEgoAgent",
                      "max_steps": "EvaluatorStepCount"}
        terminal_when = {"collision" :lambda x: x, "max_steps": lambda x : x>2}
        params = ParameterServer() # only for evaluated agents not passed to scenario!
        behaviors_tested = {"IDM": BehaviorIDMClassic(params), "Const" : BehaviorConstantAcceleration(params)}
                                        

        benchmark_runner = BenchmarkRunner(benchmark_database=db,
                                           evaluators=evaluators,
                                           terminal_when=terminal_when,
                                           behaviors=behaviors_tested,
                                           log_eval_avg_every=5)

        result = benchmark_runner.run()
        df = result.get_data_frame()
        print(df)
        self.assertEqual(len(df.index), 2*2*2) # 2 Behaviors * 2 Serialize Scenarios * 1 scenario sets

        groups = result.get_evaluation_groups()
        self.assertEqual(set(groups), set(["behavior", "scen_set"]))

    def test_database_runner_checkpoint(self):
        dbs = DatabaseSerializer(test_scenarios=4, test_world_steps=5, num_serialize_scenarios=10)
        dbs.process("data/database1")
        local_release_filename = dbs.release(version="test")

        db = BenchmarkDatabase(database_root=local_release_filename)
        evaluators = {"success" : "EvaluatorGoalReached", "collision" : "EvaluatorCollisionEgoAgent",
                      "max_steps": "EvaluatorStepCount"}
        terminal_when = {"collision" :lambda x: x, "max_steps": lambda x : x>2}
        params = ParameterServer() # only for evaluated agents not passed to scenario!
        behaviors_tested = {"IDM": BehaviorIDMClassic(params), "Const" : BehaviorConstantAcceleration(params)}
                                        

        benchmark_runner = BenchmarkRunner(benchmark_database=db,
                                           evaluators=evaluators,
                                           terminal_when=terminal_when,
                                           behaviors=behaviors_tested,
                                           log_eval_avg_every=20,
                                           checkpoint_dir="checkpoints1/")

        # one run after 30 steps benchmark dumped
        result = benchmark_runner.run(checkpoint_every = 30)
        df = result.get_data_frame()
        print(df)
        self.assertEqual(len(df.index), 40) # 2 Behaviors * 10 Serialize Scenarios * 2 scenario sets
        # check twice first, merging from checkpoints
        merged_result = BenchmarkRunner.merge_checkpoint_benchmark_results(checkpoint_dir="checkpoints1/")
        df = merged_result.get_data_frame()
        self.assertEqual(len(df.index), 30)
        # second load merged results
        self.assertTrue(os.path.exists(os.path.join("checkpoints1/merged_results.ckpnt")))
        merged_result = BenchmarkRunner.merge_checkpoint_benchmark_results(checkpoint_dir="checkpoints1/")
        df = merged_result.get_data_frame()
        self.assertEqual(len(df.index), 30)

        configs_to_run = BenchmarkRunner.get_configs_to_run(benchmark_runner.configs_to_run, merged_result)
        self.assertEqual(len(configs_to_run), 10)

        benchmark_runner2 = BenchmarkRunner(benchmark_database=db,
                                           evaluators=evaluators,
                                           terminal_when=terminal_when,
                                           behaviors=behaviors_tested,
                                           log_eval_avg_every=1,
                                           checkpoint_dir="checkpoints1/",
                                           merge_existing=True)

        result = benchmark_runner2.run(checkpoint_every = 7)
        df = result.get_data_frame()
        print(df)
        self.assertEqual(len(df.index), 40) # 2 Behaviors * 10 Serialize Scenarios * 2 scenario sets

        # check if results maintained in existing result dump, 30 from previous run + 7 after new checkpoint
        merged_result = BenchmarkRunner.merge_checkpoint_benchmark_results(checkpoint_dir="checkpoints1/")
        df = merged_result.get_data_frame()
        self.assertEqual(len(df.index), 37)

    def test_database_multiprocessing_runner(self):
        dbs = DatabaseSerializer(test_scenarios=4, test_world_steps=5, num_serialize_scenarios=5)
        dbs.process("data/database1")
        local_release_filename = dbs.release(version="test")

        db = BenchmarkDatabase(database_root=local_release_filename)
        evaluators = {"success" : "EvaluatorGoalReached", "collision" : "EvaluatorCollisionEgoAgent",
                      "max_steps": "EvaluatorStepCount"}
        terminal_when = {"collision" :lambda x: x, "max_steps": lambda x : x>2}
        params = ParameterServer() # only for evaluated agents not passed to scenario!
        behaviors_tested = {"IDM": BehaviorIDMClassic(params), "Const" : BehaviorConstantAcceleration(params)}

        benchmark_runner = BenchmarkRunnerMP(benchmark_database=db,
                                           evaluators=evaluators,
                                           terminal_when=terminal_when,
                                           behaviors=behaviors_tested,
                                           log_eval_avg_every=10)

        result = benchmark_runner.run()

        df = result.get_data_frame()
        print(df)
        self.assertEqual(len(df.index), 20) # 2 Behaviors * 5 Serialize Scenarios * 2 scenario sets

        params2 = ParameterServer()
        viewer = MPViewer(
              params=params2,
              x_range=[5060, 5160],
              y_range=[5070,5150],
              use_world_bounds=True)
        rst  = benchmark_runner.run_benchmark_config(10, viewer=viewer)

    def test_database_multiprocessing_history(self):
        dbs = DatabaseSerializer(test_scenarios=4, test_world_steps=5, num_serialize_scenarios=2)
        dbs.process("data/database1")
        local_release_filename = dbs.release(version="test")

        db = BenchmarkDatabase(database_root=local_release_filename)
        evaluators = {"success" : "EvaluatorGoalReached", "collision" : "EvaluatorCollisionEgoAgent",
                      "max_steps": "EvaluatorStepCount"}
        terminal_when = {"collision" :lambda x: x, "max_steps": lambda x : x>2}
        params = ParameterServer() # only for evaluated agents not passed to scenario!
        behaviors_tested = {"IDM": BehaviorIDMClassic(params), "Const" : BehaviorConstantAcceleration(params)}

        benchmark_runner = BenchmarkRunnerMP(benchmark_database=db,
                                           evaluators=evaluators,
                                           terminal_when=terminal_when,
                                           behaviors=behaviors_tested,
                                           log_eval_avg_every=10)
        rst = benchmark_runner.run(maintain_history=True)
        self.assertEqual(len(rst.get_histories()), 2*2*2)

        rst = benchmark_runner.run_benchmark_config(3, viewer=None, maintain_history=True)
        scenario_history = rst.get_histories()[3]
        print(scenario_history)
        params = ParameterServer()
        viewer = MPViewer(
              params=params,
              x_range=[5060, 5160],
              y_range=[5070,5150],
              use_world_bounds=True)
        viewer.drawWorld(world=scenario_history[1].GetWorldState(),
                          eval_agent_ids=scenario_history[1].eval_agent_ids)

        viewer.show(block=True)

    def test_database_multiprocessing_runner_checkpoint(self):
        dbs = DatabaseSerializer(test_scenarios=1, test_world_steps=2, num_serialize_scenarios=10)
        dbs.process("data/database1")
        local_release_filename = dbs.release(version="test")

        db = BenchmarkDatabase(database_root=local_release_filename)
        evaluators = {"success" : "EvaluatorGoalReached", "collision" : "EvaluatorCollisionEgoAgent",
                      "max_steps": "EvaluatorStepCount"}
        terminal_when = {"collision" :lambda x: x, "max_steps": lambda x : x>2}
        params = ParameterServer() # only for evaluated agents not passed to scenario!
        behaviors_tested = {"IDM": BehaviorIDMClassic(params), "Const" : BehaviorConstantAcceleration(params)}
                                        

        benchmark_runner = BenchmarkRunnerMP(benchmark_database=db,
                                           evaluators=evaluators,
                                           terminal_when=terminal_when,
                                           behaviors=behaviors_tested,
                                           log_eval_avg_every=10,
                                           num_cpus=4,
                                           checkpoint_dir="checkpoints2/",
                                           merge_existing=False)
        benchmark_runner.clear_checkpoint_dir()
        # one run after 30 steps benchmark dumped
        result = benchmark_runner.run(checkpoint_every = 3)
        df = result.get_data_frame()
        print(df)
        self.assertEqual(len(df.index), 40) # 2 Behaviors * 10 Serialize Scenarios * 2 scenario sets

        merged_result = BenchmarkRunner.merge_checkpoint_benchmark_results(checkpoint_dir="checkpoints2/")
        df = merged_result.get_data_frame()
        self.assertEqual(len(df.index), 4*9)
       # self.assertEqual(len(merged_result.get_histories()), 4*9)
        self.assertEqual(len(merged_result.get_benchmark_configs()), 4*9)

        configs_to_run = BenchmarkRunner.get_configs_to_run(benchmark_runner.configs_to_run, merged_result)
        self.assertEqual(len(configs_to_run), 4)
        ray.shutdown()
        benchmark_runner2 = BenchmarkRunnerMP(benchmark_database=db,
                                           evaluators=evaluators,
                                           terminal_when=terminal_when,
                                           behaviors=behaviors_tested,
                                           log_eval_avg_every=1,
                                           checkpoint_dir="checkpoints2/",
                                           merge_existing=True)

        result = benchmark_runner2.run(checkpoint_every = 1)
        df = result.get_data_frame()
        print(df)
        self.assertEqual(len(df.index), 40) # 2 Behaviors * 10 Serialize Scenarios * 2 scenario sets

        # check if existing result is incorporated for mergin result
        merged_result = BenchmarkRunner.merge_checkpoint_benchmark_results(checkpoint_dir="checkpoints2/")
        df = merged_result.get_data_frame()
        self.assertEqual(len(df.index), 40)


if __name__ == '__main__':
    unittest.main()


if __name__ == '__main__':
  unittest.main()
