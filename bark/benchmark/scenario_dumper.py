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
import pandas as pd
import numpy as np
import logging
from bark.benchmark.benchmark_analyzer import BenchmarkAnalyzer
from bark.runtime.viewer.video_renderer import VideoRenderer
from bark.runtime.viewer import MPViewer
from bark.runtime.commons.parameters import ParameterServer


class ScenarioDumper(BenchmarkAnalyzer):
  def __init__(self, base_result_folder, benchmark_result):
    super().__init__(benchmark_result)

    # Create result dir
    if not os.path.isdir(base_result_folder):
      Path(base_result_folder).mkdir(parents=True, exist_ok=True)
    this_result_folder = datetime.now().strftime("%d%m%Y_%H:%M:%S")
    scenario_behavior = self._benchmark_result.get_benchmark_configs()[0].scenario_set_name \
      + "_" + self._benchmark_result.get_benchmark_configs()[0].behavior_config.behavior_name
    self._result_folder = os.path.join(base_result_folder, scenario_behavior, this_result_folder)
    Path(self._result_folder).mkdir(parents=True, exist_ok=True)

  # Based on the given filter dump the matching scenarios
  def export_scenarios_filter(self, filter):
    configs_found = super().find_configs(filter)
    for config in configs_found:
      self.export(config)
    if configs_found:
      self.write_map(self._result_folder)


  # Dump a scenario given by the index in the result
  def export(self, config_idx):
    this_folder = os.path.join(self._result_folder, "ConfigIdx_"+str(config_idx))
    Path(this_folder).mkdir(parents=True, exist_ok=True)
    self.render_video(config_idx, this_folder)
    self.write_trajectory(config_idx, this_folder)
    self.write_scenario_parameter(config_idx, this_folder)
    self.write_behavior_parameter(config_idx, this_folder)

  # Write video
  def render_video(self, config_idx, folder):
    viewer = MPViewer(
      params=ParameterServer(),
      center= [375, 0],
      enforce_x_length=True,
      x_length = 100.0,
      use_world_bounds=True)
    video_exporter = VideoRenderer(renderer=viewer, world_step_time=0.2)
    super().visualize(viewer=video_exporter,  configs_idx_list=[config_idx], \
                  real_time_factor=10, fontsize=6)
    video_exporter.export_video(filename=os.path.join(folder,"video"), \
      remove_image_dir = True)

  # Collect all points from the trajectories of all agents. 
  # Note we use the historic states of the last world
  def write_trajectory(self, config_idx, folder):
    cols = ['angent_id','time','x','y','theta','v']
    table = []
    histories = super().get_benchmark_result().get_history(config_idx)
    if histories is None:
      logging.warning("No historic state saved, cannot dump trajetory")
      return
    scenario = histories[-1] #the last state inclues all the historic states
    world = scenario.GetWorldState()
    for (agent_id, agent) in world.agents.items():
      for state_action in agent.history:
        state = state_action[0]
        table.append([agent_id, state[0], state[1], state[2], state[3], state[4]])
    np_table = np.array(table)
    df = pd.DataFrame(np_table, columns=cols)
    df.to_csv(os.path.join(folder,"trajectories.csv"), sep='\t', encoding='utf-8')
    #print(df.to_string())

  def write_scenario_parameter(self, config_idx, folder):
    pass
    # todo

  def write_behavior_parameter(self, config_idx, folder):
    pass
    # todo

  def write_map(self, folder):
    pass
    #todo

