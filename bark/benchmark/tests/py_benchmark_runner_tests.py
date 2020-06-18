# Copyright (c) 2020 Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import unittest
import os
import ray

from load.benchmark_database import BenchmarkDatabase
from serialization.database_serializer import DatabaseSerializer
from bark.benchmark.benchmark_runner import BenchmarkRunner, BenchmarkConfig
from bark.benchmark.benchmark_runner_mp import BenchmarkRunnerMP, _BenchmarkRunnerActor, \
        deserialize_benchmark_config, serialize_benchmark_config

from bark.runtime.scenario.scenario_generation.configurable_scenario_generation \
  import ConfigurableScenarioGeneration

from bark.runtime.viewer.matplotlib_viewer import MPViewer

from bark.core.world.evaluation import *
from bark.runtime.commons.parameters import ParameterServer
from bark.core.models.behavior import BehaviorIDMClassic, BehaviorConstantVelocity

# to find database files
os.chdir("../benchmark_database/")

class DatabaseRunnerTests(unittest.TestCase):
    #@unittest.skip
    def test_database_runner(self):
      dbs = DatabaseSerializer(test_scenarios=2, test_world_steps=2, num_serialize_scenarios=5)
      cwd = os.getcwd()
      dbs.process("data/database1")
      local_release_filename = dbs.release(version="test")

      db = BenchmarkDatabase(database_root=local_release_filename)
      evaluators = {"success" : "EvaluatorGoalReached", "collision" : "EvaluatorCollisionEgoAgent",
                    "max_steps": "EvaluatorStepCount"}
      terminal_when = {"collision" :lambda x: x, "max_steps": lambda x : x>5}
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
      self.assertEqual(len(df.index), 10) # 2 Behaviors * 2 Serialize Scenarios * 5 scenario sets

    @unittest.skip
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

      params2 = ParameterServer()
      viewer = MPViewer(
            params=params2,
            x_range=[5060, 5160],
            y_range=[5070,5150],
            use_world_bounds=True)
      rst, _ = benchmark_runner.run_benchmark_config(10, viewer=viewer)


      rst = benchmark_runner.run(maintain_history=True)
      self.assertEqual(len(rst.get_histories()), 40)

      rst, scenario_history = benchmark_runner.run_benchmark_config(11, viewer=None, maintain_history=True)
      print(scenario_history)
      viewer = MPViewer(
            params=params2,
            x_range=[5060, 5160],
            y_range=[5070,5150],
            use_world_bounds=True)
      viewer.drawWorld(world=scenario_history[5].GetWorldState(),
                        eval_agent_ids=scenario_history[5].eval_agent_ids)

      viewer.show(block=True)


      df = result.get_data_frame()
      print(df)
      self.assertEqual(len(df.index), 40) # 2 Behaviors * 10 Serialize Scenarios * 2 scenario sets

if __name__ == '__main__':
    unittest.main()


