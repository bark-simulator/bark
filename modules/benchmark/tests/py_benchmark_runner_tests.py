# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import unittest
import os
import ray

from load.benchmark_database import BenchmarkDatabase
from serialization.database_serializer import DatabaseSerializer
from modules.benchmark.benchmark_runner import BenchmarkRunner, BenchmarkConfig
from modules.benchmark.benchmark_runner_mp import BenchmarkRunnerMP, _BenchmarkRunnerActor, \
        deserialize_benchmark_config, serialize_benchmark_config

from modules.runtime.scenario.scenario_generation.configurable_scenario_generation \
  import ConfigurableScenarioGeneration

from bark.world.evaluation import *
from modules.runtime.commons.parameters import ParameterServer
from bark.models.behavior import BehaviorIDMClassic, BehaviorConstantVelocity


class DatabaseRunnerTests(unittest.TestCase):
    @unittest.skip
    def test_database_runner(self):
        dbs = DatabaseSerializer(test_scenarios=4, test_world_steps=5, num_serialize_scenarios=10)
        os.chdir("../benchmark_database/")
        dbs.process("database")
        local_release_filename = dbs.release(version="test")

        db = BenchmarkDatabase(database_root=local_release_filename)
        evaluators = {"success" : EvaluatorGoalReached, "collision" : EvaluatorCollisionEgoAgent,
                      "max_steps": EvaluatorStepCount}
        terminal_when = {"collision" :lambda x: x, "max_steps": lambda x : x>2}
        params = ParameterServer() # only for evaluated agents not passed to scenario!
        behaviors_tested = {"IDM": BehaviorIDMClassic(params), "Const" : BehaviorConstantVelocity(params)}
                                        

        benchmark_runner = BenchmarkRunner(benchmark_database=db,
                                           evaluators=evaluators,
                                           terminal_when=terminal_when,
                                           behaviors=behaviors_tested)

        result = benchmark_runner.run()
        df = result.get_data_frame()
        print(df)
        self.assertEqual(len(df.index), 20) # 2 Behaviors * 10 Serialize Scenarios
    @unittest.skip
    def test_serialization_benchmark_config(self):
      params = ParameterServer()
      scenario_generation = ConfigurableScenarioGeneration(num_scenarios=4,params=params)
      bc = BenchmarkConfig(0, BehaviorConstantVelocity(params),
                  "const", scenario_generation.get_scenario(0), 0, "test1")
      deserialized_bc =  deserialize_benchmark_config(serialize_benchmark_config(bc))
      self.assertEqual(deserialized_bc.behavior_name, "const")
      self.assertTrue(isinstance(deserialized_bc.behavior, BehaviorConstantVelocity))
    @unittest.skip
    def test_ray_serialization_benchmark_config(self):
      ray.init(ignore_reinit_error=True)
      ray.register_custom_serializer(
          BenchmarkConfig, serializer=serialize_benchmark_config,
          deserializer=deserialize_benchmark_config)
      params = ParameterServer()
      scenario_generation = ConfigurableScenarioGeneration(num_scenarios=4,params=params)
      bc = BenchmarkConfig(0, BehaviorConstantVelocity(params),
                  "const", scenario_generation.get_scenario(0), 0, "test1")
      object_id = ray.put(bc)
      self.assertEqual(ray.get(object_id).behavior_name, "const")
      self.assertTrue(isinstance(ray.get(object_id).behavior, BehaviorConstantVelocity))
      ray.shutdown()

    def test_database_multiprocessing_runner(self):
        dbs = DatabaseSerializer(test_scenarios=4, test_world_steps=5, num_serialize_scenarios=10)
        os.chdir("../benchmark_database/")
        dbs.process("database")
        local_release_filename = dbs.release(version="test")

        db = BenchmarkDatabase(database_root=local_release_filename)
        evaluators = {"success" : "EvaluatorGoalReached", "collision" : "EvaluatorCollisionEgoAgent",
                      "max_steps": "EvaluatorStepCount"}
        terminal_when = {"collision" :lambda x: x, "max_steps": lambda x : x>2}
        params = ParameterServer() # only for evaluated agents not passed to scenario!
        behaviors_tested = {"IDM": BehaviorIDMClassic(params), "Const" : BehaviorConstantVelocity(params)}
                                        

        benchmark_runner = BenchmarkRunnerMP(benchmark_database=db,
                                           evaluators=evaluators,
                                           terminal_when=terminal_when,
                                           behaviors=behaviors_tested)

        result = benchmark_runner.run()
        
        df = result.get_data_frame()
        print(df)
        self.assertEqual(len(df.index), 20) # 2 Behaviors * 10 Serialize Scenarios

if __name__ == '__main__':
    unittest.main()


