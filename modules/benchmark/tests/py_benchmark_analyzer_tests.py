# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import unittest

from modules.benchmark.benchmark_analyzer import BenchmarkAnalyzer
from modules.benchmark.benchmark_runner import BenchmarkResult

def dummy_benchmark_results():
    return [
      {"config_idx": 1, "collision":True, "metric1": 23.5},
      {"config_idx": 2, "collision":False, "metric1": 1},
      {"config_idx": 3, "collision":False, "metric1": 23676.5},
      {"config_idx": 121, "collision":False, "metric1": 1},
      {"config_idx": 11, "collision":True, "metric1": 0.1},
      {"config_idx": 24, "collision":False, "metric1": 0.1},
    ]

class BenchmarkAnalyzerTests(unittest.TestCase):
    def test_find_config_idx(self):
        brst = BenchmarkResult(result_dict=dummy_benchmark_results(), benchmark_configs=None)
        analyzer = BenchmarkAnalyzer(benchmark_result = brst)

        configs_found = analyzer.find_configs({"collision" : lambda x : x})
        self.assertEqual(configs_found, [1, 11])



if __name__ == '__main__':
    unittest.main()