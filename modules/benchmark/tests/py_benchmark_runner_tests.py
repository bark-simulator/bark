# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import unittest
import os
import ray

try:
    import debug_settings
except:
    pass

from load.benchmark_database import BenchmarkDatabase
from serialization.database_serializer import DatabaseSerializer
from modules.benchmark.benchmark_runner import BenchmarkRunner, BenchmarkConfig
from modules.benchmark.benchmark_runner_mp import BenchmarkRunnerMP, _BenchmarkRunnerActor, \
        deserialize_benchmark_config, serialize_benchmark_config

from modules.runtime.scenario.scenario_generation.configurable_scenario_generation \
  import ConfigurableScenarioGeneration

from modules.runtime.viewer.matplotlib_viewer import MPViewer

from bark.world.evaluation import *
from modules.runtime.commons.parameters import ParameterServer
from bark.models.behavior import BehaviorIDMClassic, BehaviorConstantVelocity

try: # bazel run
  os.chdir("../benchmark_database/")
except: # debug
  os.chdir("bazel-bin/modules/benchmark/tests/py_benchmark_runner_tests.runfiles/benchmark_database")

class DatabaseRunnerTests(unittest.TestCase):
    def test_database_runner(self):
        dbs = DatabaseSerializer(test_scenarios=4, test_world_steps=5, num_serialize_scenarios=10)
        dbs.process("data/database1")
        local_release_filename = dbs.release(version="test")

        db = BenchmarkDatabase(database_root=local_release_filename)
        evaluators = {"success" : "EvaluatorGoalReached", "collision" : "EvaluatorCollisionEgoAgent",
                      "max_steps": "EvaluatorStepCount"}
        terminal_when = {"collision" :lambda x: x, "max_steps": lambda x : x>10}
        params = ParameterServer() # only for evaluated agents not passed to scenario!
        behaviors_tested = {"IDM": BehaviorIDMClassic(params), "Const" : BehaviorConstantVelocity(params)}
                                        

        benchmark_runner = BenchmarkRunner(benchmark_database=db,
                                           evaluators=evaluators,
                                           terminal_when=terminal_when,
                                           behaviors=behaviors_tested,
                                           log_eval_avg_every=1)

        result = benchmark_runner.run()
        df = result.get_data_frame()
        print(df)
        self.assertEqual(len(df.index), 40) # 2 Behaviors * 10 Serialize Scenarios * 2 scenario sets

    def test_database_runner_checkpoint(self):
        dbs = DatabaseSerializer(test_scenarios=4, test_world_steps=5, num_serialize_scenarios=10)
        dbs.process("data/database1")
        local_release_filename = dbs.release(version="test")

        db = BenchmarkDatabase(database_root=local_release_filename)
        evaluators = {"success" : "EvaluatorGoalReached", "collision" : "EvaluatorCollisionEgoAgent",
                      "max_steps": "EvaluatorStepCount"}
        terminal_when = {"collision" :lambda x: x, "max_steps": lambda x : x>10}
        params = ParameterServer() # only for evaluated agents not passed to scenario!
        behaviors_tested = {"IDM": BehaviorIDMClassic(params), "Const" : BehaviorConstantVelocity(params)}
                                        

        benchmark_runner = BenchmarkRunner(benchmark_database=db,
                                           evaluators=evaluators,
                                           terminal_when=terminal_when,
                                           behaviors=behaviors_tested,
                                           log_eval_avg_every=1,
                                           checkpoint_dir="checkpoints1/")

        # one run after 30 steps benchmark dumped
        result = benchmark_runner.run(checkpoint_every = 30)
        df = result.get_data_frame()
        print(df)
        self.assertEqual(len(df.index), 40) # 2 Behaviors * 10 Serialize Scenarios * 2 scenario sets

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
                                           checkpoint_dir="checkpoints1/")

        result = benchmark_runner2.run(checkpoint_every = 10)
        df = result.get_data_frame()
        print(df)
        self.assertEqual(len(df.index), 40) # 2 Behaviors * 10 Serialize Scenarios * 2 scenario sets

    def test_database_multiprocessing_runner(self):
        dbs = DatabaseSerializer(test_scenarios=4, test_world_steps=5, num_serialize_scenarios=10)
        dbs.process("data/database1")
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

        result = benchmark_runner.run()

        df = result.get_data_frame()
        print(df)
        self.assertEqual(len(df.index), 40) # 2 Behaviors * 10 Serialize Scenarios * 2 scenario sets

        params2 = ParameterServer()
        viewer = MPViewer(
              params=params2,
              x_range=[5060, 5160],
              y_range=[5070,5150],
              use_world_bounds=True)
        rst  = benchmark_runner.run_benchmark_config(10, viewer=viewer)

    def test_database_multiprocessing_history(self):
        dbs = DatabaseSerializer(test_scenarios=4, test_world_steps=5, num_serialize_scenarios=10)
        dbs.process("data/database1")
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
        rst = benchmark_runner.run(maintain_history=True)
        self.assertEqual(len(rst.get_histories()), 40)

        rst = benchmark_runner.run_benchmark_config(11, viewer=None, maintain_history=True)
        scenario_history = rst.get_histories()[11]
        print(scenario_history)
        params = ParameterServer()
        viewer = MPViewer(
              params=params,
              x_range=[5060, 5160],
              y_range=[5070,5150],
              use_world_bounds=True)
        viewer.drawWorld(world=scenario_history[5].get_world_state(),
                          eval_agent_ids=scenario_history[5].eval_agent_ids)

        viewer.show(block=True)

    def test_database_multiprocessing_runner_checkpoint(self):
        dbs = DatabaseSerializer(test_scenarios=4, test_world_steps=5, num_serialize_scenarios=10)
        dbs.process("data/database1")
        local_release_filename = dbs.release(version="test")

        db = BenchmarkDatabase(database_root=local_release_filename)
        evaluators = {"success" : "EvaluatorGoalReached", "collision" : "EvaluatorCollisionEgoAgent",
                      "max_steps": "EvaluatorStepCount"}
        terminal_when = {"collision" :lambda x: x, "max_steps": lambda x : x>10}
        params = ParameterServer() # only for evaluated agents not passed to scenario!
        behaviors_tested = {"IDM": BehaviorIDMClassic(params), "Const" : BehaviorConstantVelocity(params)}
                                        

        benchmark_runner = BenchmarkRunnerMP(benchmark_database=db,
                                           evaluators=evaluators,
                                           terminal_when=terminal_when,
                                           behaviors=behaviors_tested,
                                           log_eval_avg_every=1,
                                           num_cpus=4,
                                           checkpoint_dir="checkpoints2/")

        # one run after 30 steps benchmark dumped
        result = benchmark_runner.run(checkpoint_every = 3)
        df = result.get_data_frame()
        print(df)
        self.assertEqual(len(df.index), 40) # 2 Behaviors * 10 Serialize Scenarios * 2 scenario sets

        merged_result = BenchmarkRunner.merge_checkpoint_benchmark_results(checkpoint_dir="checkpoints2/")
        df = merged_result.get_data_frame()
        self.assertEqual(len(df.index), 4*9)

        configs_to_run = BenchmarkRunner.get_configs_to_run(benchmark_runner.configs_to_run, merged_result)
        self.assertEqual(len(configs_to_run), 4)
        ray.shutdown()
        benchmark_runner2 = BenchmarkRunnerMP(benchmark_database=db,
                                           evaluators=evaluators,
                                           terminal_when=terminal_when,
                                           behaviors=behaviors_tested,
                                           log_eval_avg_every=1,
                                           checkpoint_dir="checkpoints2/")

        result = benchmark_runner2.run(checkpoint_every = 5)
        df = result.get_data_frame()
        print(df)
        self.assertEqual(len(df.index), 40) # 2 Behaviors * 10 Serialize Scenarios * 2 scenario sets


if __name__ == '__main__':
    unittest.main()


