# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import time
import logging
import random
logging.getLogger().setLevel(logging.INFO)

class BenchmarkAnalyzer:
  def __init__(self, benchmark_result):
      self._benchmark_result = benchmark_result
      self._data_frame = benchmark_result.get_data_frame()

  def get_scenario_ids(self, config_idx_list):
      scenario_idxs = self._data_frame.loc[self._data_frame["config_idx"].isin(config_idx_list)]["scen_idx"]
      return list(scenario_idxs.values)

  # accepts a dict with lambda functions specifying evaluation criteria which must be fullfilled
  # e.g. evaluation_criteria={"success": lambda x: x, "collision" : lambda x : not x}
  # scenario_idx_list: a list of scenario ids, return only configs with these scenario ids
  # scenarios_as_in_configs: a list of configs ids, return only configs with scenarios of configs ids in this list
  # returns a list of config indices fullfilling these criteria
  def find_configs(self, criteria=None, scenario_idx_list=None, scenarios_as_in_configs=None):
      df_satisfied =  self._data_frame.copy()
      if criteria:
        for eval_crit, function in criteria.items():
            df_satisfied = df_satisfied.loc[df_satisfied[eval_crit].apply(function)]
      if scenarios_as_in_configs:
        scenario_idx_list = self.get_scenario_ids(scenarios_as_in_configs)
      if scenario_idx_list:
        df_satisfied = df_satisfied.loc[df_satisfied["scen_idx"].isin(scenario_idx_list)]

      configs_found = list(df_satisfied["config_idx"].values)
      configs_found.sort()
      return configs_found

  def visualize(self, configs_idx_list, viewer, viewer_names=None, real_time_factor=1.0, display_info=True, **kwargs):
      if not all(isinstance(elem, list) for elem in configs_idx_list):
          configs_idx_list = [configs_idx_list]
      if not isinstance(viewer, list):
          viewer = [viewer]
      if not len(configs_idx_list) == len(viewer):
          raise ValueError("Equal number of viewers and configs_idx_list required.")

      num_configs = len(configs_idx_list[0])
      num_parallel_visualizations = len(viewer)
      for config_list_idx in range(0, num_configs):
          # collect histories
          histories_collected = {}
          info_strings_collected = {}
          sim_times_collected = {}
          for viewer_idx in range(0, num_parallel_visualizations):
              config_idx = configs_idx_list[viewer_idx][config_list_idx]
              benchmark_config = self._benchmark_result.get_benchmark_config(config_idx)
              histories = self._benchmark_result.get_history(config_idx)
              histories_collected[viewer_idx] = histories

              if len(histories) == 0:
                logging.info("No histories for config idx {}. Skipping....".format(config_idx))

              if real_time_factor:
                  try:
                      sim_time = benchmark_config.scenario.json_params["Simulation"]["StepTime"]
                  except:
                      logging.warning("Simulation step time not privided for config {}. Using default 0.2s".format(config_idx))
                      sim_time = 0.2

              info_text_list = benchmark_config.get_info_string_list()
              info_text = " | ".join(info_text_list)
              info_strings_collected[viewer_idx] = info_text

              sim_times_collected[viewer_idx] = sim_time

          self._render_histories(histories_collected, info_strings_collected, viewer,
                               viewer_names, sim_time, real_time_factor, display_info,  **kwargs)


  def _render_histories(self, histories_collected, info_strings_collected, viewer_list,
                        viewer_names, sim_time, real_time_factor, display_info, **kwargs):
          world_time = 0
          step_time = sim_time/real_time_factor
          steps_to_go = True
          step = 0
          while steps_to_go:
              step += 1
              steps_to_go = False
              for viewer_idx in range(0, len(viewer_list)):
                  histories = histories_collected[viewer_idx]
                  if len(histories) < step:
                    continue
                  steps_to_go = True
                  scenario = histories[step-1]
                  viewer = viewer_list[viewer_idx]
                  world = scenario.get_world_state()
                  world.time = world_time
                  if display_info and not viewer_names:
                      info_text = info_strings_collected[viewer_idx]
                      viewer.drawText(text=info_text, position=(0.5,1.05), **kwargs)
                  elif viewer_names:
                      viewer.drawText(text=viewer_names[viewer_idx], position=(0.5,1.05), **kwargs)
                  viewer.drawWorld(world = world,
                              eval_agent_ids = scenario.eval_agent_ids, \
                              scenario_idx = None, debug_text=False)
                  viewer_list[viewer_idx].show()
              world_time += sim_time
              if real_time_factor:
                  time.sleep(step_time)
              for viewer_idx in range(0, len(viewer_list)):
                  viewer_list[viewer_idx].clear()