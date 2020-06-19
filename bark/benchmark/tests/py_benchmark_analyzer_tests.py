# Copyright (c) 2020 Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import unittest

from bark.benchmark.benchmark_analyzer import BenchmarkAnalyzer
from bark.benchmark.benchmark_runner import BenchmarkResult

def dummy_benchmark_results():
    return [
      {"config_idx": 1, "collision":True, "metric1": 23.5, "behavior": "test", "scen_idx": 10},
      {"config_idx": 2, "collision":False, "metric1": 1, "behavior": "test2", "scen_idx": 12},
      {"config_idx": 3, "collision":False, "metric1": 23676.5, "behavior": "test1", "scen_idx": 3},
      {"config_idx": 121, "collision":False, "metric1": 1, "behavior": "test2", "scen_idx": 4},
      {"config_idx": 11, "collision":True, "metric1": 0.1, "behavior": "test", "scen_idx": 5},
      {"config_idx": 24, "collision":False, "metric1": 0.1, "behavior": "test1", "scen_idx": 7},
    ]

class BenchmarkAnalyzerTests(unittest.TestCase):
    def test_find_config_idx(self):
        brst = BenchmarkResult(result_dict=dummy_benchmark_results(), benchmark_configs=None)
        analyzer = BenchmarkAnalyzer(benchmark_result = brst)

        configs_found = analyzer.find_configs({"collision" : lambda x : x})
        self.assertEqual(configs_found, [1, 11])

        configs_found = analyzer.find_configs({"collision" : lambda x : not x})
        self.assertEqual(configs_found, [2, 3, 24, 121])

        configs_found = analyzer.find_configs({"metric1" : lambda x : x == 0.1})
        self.assertEqual(configs_found, [11, 24])

        configs_found = analyzer.find_configs({"collision" : lambda x : not x, "metric1" : lambda x : x == 0.1})
        self.assertEqual(configs_found, [24])

        configs_found = analyzer.find_configs({"collision" : lambda x : x, "metric1" : lambda x : x > 1})
        self.assertEqual(configs_found, [1])

        configs_found = analyzer.find_configs({"collision" : lambda x : x, "behavior" : lambda x : x=="test2"})
        self.assertEqual(configs_found, [])

        configs_found = analyzer.find_configs(scenario_idx_list=[12, 3 , 5])
        self.assertEqual(configs_found, [2, 3, 11])

        configs_found = analyzer.find_configs({"collision" : lambda x : not x}, scenario_idx_list=[12, 3 , 5])
        self.assertEqual(configs_found, [2, 3])

        configs_found = analyzer.find_configs({"collision" : lambda x : x}, scenarios_as_in_configs=[24, 11 , 121, 1])
        self.assertEqual(configs_found, [1, 11])


if __name__ == '__main__':
    unittest.main()