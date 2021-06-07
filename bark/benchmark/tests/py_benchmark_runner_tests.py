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
from bark.benchmark.benchmark_result import BenchmarkResult
from bark.benchmark.benchmark_runner import BenchmarkRunner, BenchmarkConfig, EvaluationConfig
from bark.benchmark.benchmark_runner_mp import BenchmarkRunnerMP, _BenchmarkRunnerActor, \
  deserialize_benchmark_config, serialize_benchmark_config

from bark.runtime.viewer.matplotlib_viewer import MPViewer

from bark.core.world.evaluation import *
from bark.core.world.evaluation.ltl import ConstantLabelFunction
from bark.runtime.commons.parameters import ParameterServer
from bark.core.models.behavior import BehaviorIDMClassic, BehaviorConstantAcceleration
from bark.world.tests.python_behavior_model import PythonDistanceBehavior

from bark.benchmark.tests.test_evaluator import TestPythonEvaluatorSerializable

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
        safe_dist_params = ParameterServer(log_if_default=True)
        test_python_params = ParameterServer(log_if_default=True)
        evaluators = {"success" : "EvaluatorGoalReached", "collision" : "EvaluatorCollisionEgoAgent",
                      "max_steps": "EvaluatorStepCount", "safe_dist_lon" : {"type" : "EvaluatorDynamicSafeDist", "params" : safe_dist_params},
                      "safe_dist_lat" : {"type" : "EvaluatorStaticSafeDist", "params" : safe_dist_params},
                      "test_evaluator" : {"type" : "TestPythonEvaluator", "params" : test_python_params},
                      "test_evaluator_serializable" : TestPythonEvaluatorSerializable()}
        terminal_when = {"collision" :lambda x: x, "max_steps": lambda x : x>2, "safe_dist_lon" : lambda x: x }
        params = ParameterServer() # only for evaluated agents not passed to scenario!
        behaviors_tested = {"IDM": BehaviorIDMClassic(params), "Const" : BehaviorConstantAcceleration(params)}
                                        
        EvaluationConfig.AddEvaluationModule("bark.benchmark.tests.test_evaluator")
        benchmark_runner = BenchmarkRunner(benchmark_database=db,
                                           evaluators=evaluators,
                                           log_eval_avg_every=1,
                                           terminal_when=terminal_when,
                                           behaviors=behaviors_tested)

        result = benchmark_runner.run()
        df = result.get_data_frame()
        print(df)
        self.assertEqual(len(df.index), 2*2*2) # 2 Behaviors * 2 Serialize Scenarios * 1 scenario sets
        result.load_benchmark_configs()
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
        benchmark_runner.clear_checkpoint_dir()
        # one run after 30 steps benchmark dumped
        result = benchmark_runner.run(checkpoint_every = 30)
        df = result.get_data_frame()
        print(df)
        self.assertEqual(len(df.index), 40) # 2 Behaviors * 10 Serialize Scenarios * 2 scenario sets
        # check twice first, merging from checkpoints
        merged_result = BenchmarkRunner.merge_checkpoint_benchmark_results(checkpoint_dir="checkpoints1/")
        df = merged_result.get_data_frame()
        self.assertEqual(len(df.index), 40)

    def test_database_multiprocessing_runner(self):
        dbs = DatabaseSerializer(test_scenarios=4, test_world_steps=5, num_serialize_scenarios=5)
        dbs.process("data/database1")
        local_release_filename = dbs.release(version="test")

        db = BenchmarkDatabase(database_root=local_release_filename)
        test_python_params = ParameterServer(log_if_default=True)
        evaluators = {"success" : "EvaluatorGoalReached", "collision" : "EvaluatorCollisionEgoAgent",
                      "max_steps": "EvaluatorStepCount",
                      "test_evaluator" : {"type" : "TestPythonEvaluator", "params" : test_python_params}}
        terminal_when = {"collision" :lambda x: x, "max_steps": lambda x : x>2}
        params = ParameterServer() # only for evaluated agents not passed to scenario!
        behaviors_tested = {"IDM": BehaviorIDMClassic(params), "Const" : BehaviorConstantAcceleration(params)}

        EvaluationConfig.AddEvaluationModule("bark.benchmark.tests.test_evaluator")
        benchmark_runner = BenchmarkRunnerMP(benchmark_database=db,
                                           evaluators=evaluators,
                                           terminal_when=terminal_when,
                                           behaviors=behaviors_tested,
                                           log_eval_avg_every=10)
        benchmark_runner.clear_checkpoint_dir()
        result = benchmark_runner.run()

        df = result.get_data_frame()
        print(df)
        self.assertEqual(len(df.index), 20) # 2 Behaviors * 5 Serialize Scenarios * 2 scenario sets

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
        benchmark_runner.clear_checkpoint_dir()
        rst = benchmark_runner.run(maintain_history=True)
        rst.load_histories()
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
    @unittest.skip
    def test_database_runner_python_behavior(self):
          dbs = DatabaseSerializer(test_scenarios=4, test_world_steps=5, num_serialize_scenarios=2)
          dbs.process("data/database1")
          local_release_filename = dbs.release(version="test")

          db = BenchmarkDatabase(database_root=local_release_filename)
          safe_dist_params = ParameterServer(log_if_default=True)
          evaluators = {"success" : "EvaluatorGoalReached", "collision" : "EvaluatorCollisionEgoAgent",
                        "max_steps": "EvaluatorStepCount", "safe_dist_lon" : {"type" : "EvaluatorDynamicSafeDist", "params" : safe_dist_params},
                        "safe_dist_lat" : {"type" : "EvaluatorStaticSafeDist", "params" : safe_dist_params}}
          terminal_when = {"collision" :lambda x: x, "max_steps": lambda x : x>2, "safe_dist_lon" : lambda x: x }
          params = ParameterServer() # only for evaluated agents not passed to scenario!
          behaviors_tested = {"python_behavior": PythonDistanceBehavior(params)}
                                          

          benchmark_runner = BenchmarkRunner(benchmark_database=db,
                                            evaluators=evaluators,
                                            terminal_when=terminal_when,
                                            behaviors=behaviors_tested,
                                            log_eval_avg_every=5, deepcopy=False)
          benchmark_runner.clear_checkpoint_dir()
          result = benchmark_runner.run()
          df = result.get_data_frame()
          print(df)
          self.assertEqual(len(df.index), 1*2*2) # 2 Behaviors * 2 Serialize Scenarios * 1 scenario sets
          result.load_benchmark_configs()
          groups = result.get_evaluation_groups()
          self.assertEqual(set(groups), set(["behavior", "scen_set"]))

if __name__ == '__main__':
    unittest.main()


if __name__ == '__main__':
  unittest.main()
