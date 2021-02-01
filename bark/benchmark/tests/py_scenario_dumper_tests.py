# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import unittest
import numpy as np
import os
import shutil

from bark.benchmark.scenario_dumper import ScenarioDumper
from bark.benchmark.benchmark_result import BenchmarkResult, BenchmarkConfig, BehaviorConfig
from bark.runtime.viewer.matplotlib_viewer import MPViewer

def dummy_benchmark_results():
    br = [
      {"config_idx": 24,  "collision":False, "rss": True, "behavior": "test1", "scen_idx": 12},
      {"config_idx": 1,   "collision":True, "rss": False, "behavior": "test", "scen_idx": 10},
      {"config_idx": 500, "collision":True, "rss": True, "behavior": "test", "scen_idx": 112},
    ]
    behaviorc = BehaviorConfig("behavior_name", "behavior")
    benchmarkc = BenchmarkConfig(0, behaviorc, "test scenario", 0, "scenario_set_name")
    return BenchmarkResult(result_dict=br, benchmark_configs=[benchmarkc])

class ScenarioDumperTest(unittest.TestCase):
  def test_creation(self):
    basepath = "/tmp/test12345"
    if(os.path.isdir(basepath)):
      shutil.rmtree(basepath)
    sd = ScenarioDumper(basepath, dummy_benchmark_results())
    self.assertTrue(os.path.isdir(basepath))

  def test_filter(self):
    sd = ScenarioDumper("/tmp/test12345", dummy_benchmark_results())
    configs_found = sd.find_configs({"collision" : lambda x : x, "rss" : lambda x : not x})
    self.assertEqual(configs_found, [1])

  def test_export(self):
    sd = ScenarioDumper("/tmp/test12345", dummy_benchmark_results())
    sd.export(1)

  def test_filtered_export1(self):
    sd = ScenarioDumper("/tmp/test12345", dummy_benchmark_results())
    ret = sd.export_scenarios_filter(filter = {"collision" : lambda x : x, "rss" : lambda x : not x})
    self.assertEqual(ret, [1])

  def test_filtered_export2(self):
    sd = ScenarioDumper("/tmp/test12345", dummy_benchmark_results())
    ret = sd.export_scenarios_filter(filter = {"collision" : lambda x : x, "rss" : lambda x : not x}, config_idx_list = [1, 2, 3, 24])
    self.assertEqual(ret, [1])

  def test_filtered_export3(self):
    sd = ScenarioDumper("/tmp/test12345", dummy_benchmark_results())
    ret = sd.export_scenarios_filter(config_idx_list = [1, 24])
    self.assertEqual(ret, [1, 24])

  def test_filtered_export4(self):
    sd = ScenarioDumper("/tmp/test12345", dummy_benchmark_results())
    ret = sd.export_scenarios_filter(filter = {"collision" : lambda x : x, "rss" : lambda x : not x}, config_idx_list = [2444])
    self.assertEqual(ret, [])

  def test_filtered_export5(self):
    sd = ScenarioDumper("/tmp/test12345", dummy_benchmark_results())
    self.assertRaises(ValueError, sd.export_scenarios_filter)

if __name__ == '__main__':
  unittest.main()
