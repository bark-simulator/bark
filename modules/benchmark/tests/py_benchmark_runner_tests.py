# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import unittest
import os

from load.benchmark_database import BenchmarkDatabase
from serialization.database_serializer import DatabaseSerializer
from modules.benchmark.benchmark_runner import BenchmarkRunner

from bark.world.evaluation import *
from modules.runtime.commons.parameters import ParameterServer
from bark.models.behavior import BehaviorIDMClassic, BehaviorConstantVelocity


class DatabaseRunnerTests(unittest.TestCase):
    def test_database_local_release(self):
        dbs = DatabaseSerializer(test_scenarios=10, test_world_steps=5, num_serialize_scenarios=10)
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
        
        print(result.get_data_frame())

if __name__ == '__main__':
    unittest.main()


