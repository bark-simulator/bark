# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT



class BenchmarkAnalyzer:
  def __init__(self, benchmark_result):
      self._benchmark_result = benchmark_result
      self._data_frame = benchmark_result.get_data_frame()

  # accepts a dict with lambda functions specifying evaluation criteria which must be fullfilled
  # e.g. evaluation_criteria={"success": lambda x: x, "collision" : lambda x : not x}
  # returns a list of config indices fullfilling these criteria
  def find_configs(self, evaluation_criteria, behavior=None):
      df_satisfied =  self._data_frame.copy()
      if behavior:
        df_satisfied = df_satisfied.loc[df_satisfied["behavior"] == behavior]

      for eval_crit, function in evaluation_criteria.items():
          df_satisfied = df_satisfied.loc[df_satisfied[eval_crit].apply(function)]
      return list(df_satisfied["config_idx"].values)


