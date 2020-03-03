# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import time
import logging
logging.getLogger().setLevel(logging.INFO)

class BenchmarkAnalyzer:
  def __init__(self, benchmark_result):
      self._benchmark_result = benchmark_result
      self._data_frame = benchmark_result.get_data_frame()

  # accepts a dict with lambda functions specifying evaluation criteria which must be fullfilled
  # e.g. evaluation_criteria={"success": lambda x: x, "collision" : lambda x : not x}
  # returns a list of config indices fullfilling these criteria
  def find_configs(self, criteria):
      df_satisfied =  self._data_frame.copy()
      for eval_crit, function in criteria.items():
          df_satisfied = df_satisfied.loc[df_satisfied[eval_crit].apply(function)]
      configs_found = list(df_satisfied["config_idx"].values)
      configs_found.sort()
      return configs_found

  def visualize(self, criteria, viewer, real_time_factor=1.0):
      configs_found = self.find_configs(criteria)
      for config_idx in configs_found:
          benchmark_config = self._benchmark_result.get_benchmark_config(config_idx)
          histories = self._benchmark_result.get_history(config_idx)
          if len(histories) == 0:
             logging.info("No histories for config idx {}. Skipping....".format(config_idx))

          if real_time_factor:
              try:
                  sim_time = benchmark_config.scenario.json_params["Simulation"]["StepTime"]
              except:
                  logging.warning("Simulation step time not privided for config {}. Using default 0.2s".format(config_idx))
                  sim_time = 0.2
          step_time =  sim_time/real_time_factor
          world_time = 0
          for scenario in histories:
              world = scenario.get_world_state()
              world.time = world_time
              viewer.drawWorld(world = world,
                          eval_agent_ids = benchmark_config.scenario.eval_agent_ids, \
                          scenario_idx = benchmark_config.scenario_idx)
              viewer.show()
              world_time += sim_time
              if real_time_factor:
                  time.sleep(step_time)
              viewer.clear()