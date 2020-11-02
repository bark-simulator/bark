# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

from pathlib import Path
import os
from datetime import datetime
from bark.benchmark.benchmark_analyzer import BenchmarkAnalyzer
from bark.runtime.viewer import MPViewer


###### TODO ########
# Scenario Writer (Tobias) für eine oder mehrere Configs
#     Output:
#         Ordner mit Video,
#         text-file mit Trajektorien,
#         Karte,
#         bestehendes szenario parameter file für benchmark,
#         behavior model parameter file



class ScenarioDumper(BenchmarkAnalyzer):
  def __init__(self, base_result_folder, benchmark_result):
    super().__init__(benchmark_result)

    # Create result dir
    if not os.path.isdir(base_result_folder):
      Path(base_result_folder).mkdir(parents=True, exist_ok=True)
    this_result_folder = datetime.now().strftime("%d%m%Y_%H:%M:%S")
    scenario_behavior = self._benchmark_result.get_benchmark_configs().scenario_set_name + "_" + self._benchmark_result.get_benchmark_configs().behavior_config.behavior_name
    self._result_folder = os.path.join(base_result_folder, scenario_behavior, this_result_folder)
    Path(self._result_folder).mkdir(parents=True, exist_ok=True)

  def export_scenarios_filter(self, filter):
    configs_found = analyzer.find_configs(filter)
    for config in configs_found:
      self.export(config_idx)
    if config_found:
      self.write_map(self._result_folder)

  def export(self, config_idx):
    this_folder = os.path.join(self._result_folder, "ConfigIdx"+str(config_idx))
    Path(this_folder).mkdir(parents=True, exist_ok=True)
    self.render_video(config_idx, this_folder)
    self.write_trajectory(config_idx, this_folder)
    self.write_scenario_parameter(config_idx, this_folder)
    self.write_behavior_parameter(config_idx, this_folder)

  def render_video(self, config_idx, folder):
    viewer = MPViewer()
    # super().visualize([config_idx], viewer)
    # store....

  def write_trajectory(self, config_idx, folder):
    pass
    # todo

  def write_scenario_parameter(self, config_idx, folder):
    pass
    # todo

  def write_behavior_parameter(self, config_idx, folder):
    pass
    # todo

  def write_map(self, folder):
    pass
    #todo

