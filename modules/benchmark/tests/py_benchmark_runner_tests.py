# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import unittest

from load.benchmark_database import BenchmarkDatabase
from modules.benchmark.benchmark_runner import BenchmarkRunner

from bark.world.evaluation import *
from modules.runtime.commons.parameters import ParameterServer
from bark.models.behavior import BehaviorIDMClassic, BehaviorConstantVelocity


class DatabaseRunnerTests(unittest.TestCase):
    def test_database_from_github_release(self):                                                 
        db = BenchmarkDatabase(database_root="external/benchmark_database_release")
        evaluators = {"success" : EvaluatorGoalReached, "collision" : EvaluatorCollisionEgoAgent,
                      "max_steps": EvaluatorStepCount}
        terminal_when = {"collision" :lambda x: x, "max_steps": lambda x : x>2}
        params = ParameterServer() # only for evaluated agents not passed to scenario!
        behaviors_tested = {"IDM": BehaviorIDMClassic(params), "Const" : BehaviorConstantVelocity(params)}
                                        

        benchmark_runner = BenchmarkRunner(benchmark_database=db,
                                           evaluators=evaluators,
                                           terminal_when=terminal_when,
                                           behaviors=behaviors_tested)

        #benchmark_runner.run(1)
        print(benchmark_runner.dataframe.to_string())
        

if __name__ == '__main__':
    unittest.main()


