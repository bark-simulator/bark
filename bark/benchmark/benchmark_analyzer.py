# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import time
import logging
import random
logging.getLogger().setLevel(logging.INFO)

class BenchmarkAnalyzer:
  def __init__(self, benchmark_result):
      self._benchmark_result = benchmark_result
      self._data_frame = benchmark_result.get_data_frame()

  def get_benchmark_result(self):
    return self._benchmark_result

  def get_scenario_ids(self, config_idx_list):
      scenario_idxs = self._data_frame.loc[self._data_frame["config_idx"].isin(config_idx_list)]
      scen_df_copy = scenario_idxs.reset_index()
      scen_df_copy = scen_df_copy.reindex(scen_df_copy.config_idx.map( \
                    {x: i for i, x in enumerate(config_idx_list)}).sort_values().index)
      return list(scen_df_copy.scen_idx.values)

  # accepts a dict with lambda functions specifying evaluation criteria which must be fullfilled
  # e.g. evaluation_criteria={"success": lambda x: x, "collision" : lambda x : not x}
  # scenario_idx_list: a list of scenario ids, return only configs with these scenario ids
  # scenarios_as_in_configs: a list of configs ids, return only configs with scenarios of configs ids in this list
  # returns a list of config indices fullfilling these criteria
  def find_configs(self, criteria=None, scenario_idx_list=None, scenarios_as_in_configs=None, in_configs=None):
      df_satisfied =  self._data_frame.copy()
      if criteria:
        for eval_crit, function in criteria.items():
            df_satisfied = df_satisfied.loc[df_satisfied[eval_crit].apply(function)]
      if scenarios_as_in_configs:
            scenario_idx_list = self.get_scenario_ids(scenarios_as_in_configs)
      if in_configs:
            df_satisfied = df_satisfied.loc[df_satisfied["config_idx"].isin(in_configs)]
      if scenario_idx_list:
            df_satisfied = df_satisfied.loc[df_satisfied["scen_idx"].isin(scenario_idx_list)]
            found_scen_idx = [scen_idx for scen_idx in scenario_idx_list if scen_idx in \
                        list(df_satisfied["scen_idx"].values)]
            df_satisfied.reset_index(inplace=True)
            df_satisfied = df_satisfied.reindex(df_satisfied.scen_idx.map( \
                    {x: i for i, x in enumerate(found_scen_idx)}).sort_values().index)

      configs_found = list(df_satisfied["config_idx"].values)
      return configs_found

  def make_scenarios_congruent(self, configs_idx_lists):
      matching_scenarios = set(self.get_scenario_ids(configs_idx_lists[0]))
      for configs_idx_list in configs_idx_lists[1:]:
          matching_scenarios = matching_scenarios & set(self.get_scenario_ids(configs_idx_list))
      matching_scenarios = list(matching_scenarios)
      congruent_config_lists = []
      matching_scenarios.sort()
      for config_idx_list in configs_idx_lists:
          congruent_list = self.find_configs(scenario_idx_list=matching_scenarios, \
                                    in_configs=config_idx_list)
          scenarios_in_conf = self.get_scenario_ids(congruent_list)
          assert(scenarios_in_conf == matching_scenarios)
          congruent_config_lists.append(congruent_list)
      return congruent_config_lists

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

              if benchmark_config is None :
                return

              if histories is None or len(histories) == 0:
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
          step_time = sim_time/real_time_factor
          steps_to_go = True
          step = 0
          while steps_to_go:
              step += 1
              steps_to_go = False
              for viewer_idx in range(0, len(viewer_list)):
                  histories = histories_collected[viewer_idx]
                  history_idx = step - 1
                  if len(histories) < step:
                    history_idx = len(histories) -1
                  else:
                    steps_to_go = True
                  scenario = histories[history_idx]
                  viewer = viewer_list[viewer_idx]
                  world = scenario.GetWorldState()
                  world.time = sim_time*history_idx
                  if display_info and not viewer_names:
                      info_text = info_strings_collected[viewer_idx]
                      viewer.drawText(text=info_text, position=(0.5,1.05), **kwargs)
                  elif viewer_names:
                      viewer.drawText(text=viewer_names[viewer_idx], position=(0.5,1.02), **kwargs)
                      viewer.drawText(text="$t={:.1f}$".format(world.time), position=(0.05, 0.05), horizontalalignment="left", \
                            verticalalignment="bottom", fontsize=kwargs.get("fontsize")*0.5 or 3.0)
                  viewer.drawWorld(world = world,
                              eval_agent_ids = scenario.eval_agent_ids, \
                              scenario_idx = None, debug_text=False)
                  viewer_list[viewer_idx].show()
              if real_time_factor:
                  time.sleep(step_time)
              for viewer_idx in range(0, len(viewer_list)):
                  viewer_list[viewer_idx].clear()