# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

try:
    import debug_settings
except:
    pass

import unittest

from bark.benchmark.benchmark_analyzer import BenchmarkAnalyzer
from bark.benchmark.benchmark_result import BenchmarkResult

def dummy_benchmark_results():
    return [
      {"config_idx": 24, "collision":False, "metric1": 0.1, "behavior": "test1", "scen_idx": 12},
      {"config_idx": 1, "collision":True, "metric1": 23.5, "behavior": "test", "scen_idx": 10},
      {"config_idx": 500, "collision":True, "metric1": 23.5, "behavior": "test", "scen_idx": 112},
      {"config_idx": 2, "collision":False, "metric1": 1, "behavior": "test2", "scen_idx": 12},
      {"config_idx": 41, "collision":False, "metric1": 0.2, "behavior": "test1", "scen_idx": 12},
      {"config_idx": 3, "collision":False, "metric1": 23676.5, "behavior": "test1", "scen_idx": 3},
      {"config_idx": 12, "collision":False, "metric1": 0.4, "behavior": "test1", "scen_idx": 5},
      {"config_idx": 121, "collision":False, "metric1": 3, "behavior": "test2", "scen_idx": 4},
      {"config_idx": 11, "collision":True, "metric1": 0.1, "behavior": "test", "scen_idx": 5},
      {"config_idx": 35, "collision":False, "metric1": 0.6, "behavior": "test1", "scen_idx": 5},
      {"config_idx": 42, "collision":False, "metric1": 0.4, "behavior": "test1", "scen_idx": 7},
    ]

class BenchmarkAnalyzerTests(unittest.TestCase):
    def test_find_config_idx(self):
        brst = BenchmarkResult(result_dict=dummy_benchmark_results(), benchmark_configs=None)
        analyzer = BenchmarkAnalyzer(benchmark_result = brst)

        configs_found = analyzer.find_configs({"collision" : lambda x : x})
        self.assertEqual(configs_found, [1, 500, 11])

        configs_found = analyzer.find_configs({"collision" : lambda x : not x})
        self.assertEqual(configs_found, [24, 2, 41, 3, 12, 121, 35, 42])

        configs_found = analyzer.find_configs({"metric1" : lambda x : x == 0.1})
        self.assertEqual(configs_found, [24, 11])

        configs_found = analyzer.find_configs({"collision" : lambda x : not x, "metric1" : lambda x : x == 0.1})
        self.assertEqual(configs_found, [24])

        configs_found = analyzer.find_configs({"collision" : lambda x : x, "metric1" : lambda x : x > 1})
        self.assertEqual(configs_found, [1, 500])

        configs_found = analyzer.find_configs({"collision" : lambda x : x, "behavior" : lambda x : x=="test2"})
        self.assertEqual(configs_found, [])

        configs_found = analyzer.find_configs(scenario_idx_list=[4, 10, 7])
        self.assertEqual(configs_found, [121, 1, 42])

        configs_found = analyzer.find_configs({"collision" : lambda x : not x}, scenario_idx_list=[4, 10, 7])
        self.assertEqual(configs_found, [121, 42])

        configs_found = analyzer.find_configs({"collision" : lambda x : x}, scenarios_as_in_configs=[24, 11 , 121, 1])
        self.assertEqual(configs_found, [11, 1])

        configs_found = analyzer.find_configs(scenario_idx_list=[4, 10, 7], in_configs=[1, 42])
        self.assertEqual(configs_found, [1, 42])

    def test_make_scenarios_congruent(self):
        brst = BenchmarkResult(result_dict=dummy_benchmark_results(), benchmark_configs=None)
        analyzer = BenchmarkAnalyzer(benchmark_result = brst)

        # scenarios 5 and 12 in all lists
        congruent_list = analyzer.make_scenarios_congruent(configs_idx_lists = [
          [3, 2, 500, 11], [35, 1 , 41], [500, 41, 12, 121]
        ])
        self.assertEqual(congruent_list[0], [11, 2])

if __name__ == '__main__':
    unittest.main()