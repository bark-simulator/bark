# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import os
import pickle
import pandas as pd
import logging
import copy
import time
import glob
import re

logging.basicConfig()
logging.getLogger().setLevel(logging.INFO)

from modules.runtime.commons.parameters import ParameterServer
from modules.runtime.scenario.scenario import Scenario

# contains information specifying
class BehaviorConfig:
    def __init__(self, behavior_name, behavior, param_descriptions=None):
        self.behavior_name = behavior_name
        self.behavior = behavior
        self.param_descriptions = param_descriptions or {}

    def as_dict(self):
        dct = {"behavior": self.behavior_name, **self.param_descriptions}
        return dct

    @staticmethod
    def configs_from_dict(behavior_dict):
        behavior_configs = []
        for behavior_name, behavior in behavior_dict.items():
            config = BehaviorConfig(behavior_name, behavior)
            behavior_configs.append(config)
        return behavior_configs

# contains information for a single benchmark run
class BenchmarkConfig:
    def __init__(self, config_idx, behavior_config,
                 scenario, scenario_idx, scenario_set_name):
        self.config_idx = config_idx
        self.behavior_config = behavior_config
        self.scenario = scenario
        self.scenario_idx = scenario_idx
        self.scenario_set_name = scenario_set_name

    def get_info_string_list(self):
        info_strings = ["ConfigIdx: {}".format(self.config_idx),
                        "Behavior: {}".format(self.behavior_config.behavior_name),
                        "ScenarioSet: {}".format(self.scenario_set_name),
                        "ScenarioIdx: {}".format(self.scenario_idx)]
        return info_strings

    def as_dict(self):
        return {"config_idx": self.config_idx,
                "scen_set": self.scenario_set_name,
                "scen_idx": self.scenario_idx,
                **self.behavior_config.as_dict()}

    def get_evaluation_groups(self):
      return ["scen_set", *list(self.behavior_config.as_dict().keys())]


# result of benchmark run
class BenchmarkResult:
    def __init__(self, result_dict = None, benchmark_configs = None, histories = None):
        self.__result_dict = result_dict or []
        self.__benchmark_configs = benchmark_configs or []
        self.__data_frame = None
        self.__histories = histories or {}

    def get_data_frame(self):
        if not isinstance(self.__data_frame, pd.DataFrame):
            self.__data_frame = pd.DataFrame(self.__result_dict)
        return self.__data_frame

    def get_result_dict(self):
        return self.__result_dict

    def get_benchmark_configs(self):
        return self.__benchmark_configs

    def get_histories(self):
        return self.__histories

    def get_benchmark_config(self, config_idx):
        return BenchmarkResult.find_benchmark_config(
            self.__benchmark_configs, config_idx)

    def get_benchmark_config_indices(self):
        return [bc.config_idx for bc in self.__benchmark_configs]

    def get_history(self, config_idx):
        return self.__histories[config_idx]

    def get_evaluation_groups(self):
        evaluation_groups = {"scen_set"}
        for conf in self.__benchmark_configs:
            evaluation_groups.update(set(conf.get_evaluation_groups()))
        return list(evaluation_groups)

    @staticmethod
    def find_benchmark_config(benchmark_configs, config_idx):
        BenchmarkResult._sort_bench_confs(benchmark_configs)
        bench_conf = benchmark_configs[config_idx]
        assert (bench_conf.config_idx == config_idx)
        return bench_conf

    @staticmethod
    def _sort_bench_confs(benchmark_configs):
        def sort_key(bench_conf):
            return bench_conf.config_idx
        benchmark_configs.sort(key=sort_key)

    @staticmethod
    def load(filename):
        with open(filename, 'rb') as handle:
            dmp = pickle.load(handle)
        return dmp

    @staticmethod
    def load_results(filename):
        pass

    @staticmethod
    def load_histories(file_name, config_idx_list):
        pass

    @staticmethod
    def load_benchmark_configs(file_name, config_idx_list):
        pass

    @staticmethod
    def _load_and_merge(zip_file_handle, filetype, config_idx_list):
        total_file_list = [filename for filename in zip_file_handle.namelist() \
                         if filetype in filename]
        merged_iterable = None
        files_to_load = []
        for file in total_file_list:
            idx_range = re.findall("config_idx_[0-9]+_to_[0-9]+.{}".format(filetype), file)
            if len(idx_range) < 1:
                continue
            found_config = filter(lambda conf_idx: config_idx >= idx_range[0] \
                                 and config_idx <= idx_range[0], config_idx_list)
            if len(found_config) > 0:
                files_to_load.extend(file)

        for file in files_to_load:
            bytes = zip_file_handle.read(file)
            iterable = pickle.loads(bytes)
            if isinstance(iterable, list):
                if not merged_iterable:
                    merged_iterable = []
                merged_iterable.extend(iterable)
            elif isinstance(iterable, dict):
                if not merged_iterable:
                    merged_iterable = {}
                merged_iterable.update(iterable)
        return merged_iterable

    @staticmethod
    def _save_and_split(zip_file_handle, filetype, pickable_iterable, max_bytes_per_file):
        whole_list_byte_size = len(pickle.dumps(pickable_iterable))
        num_files = math.ceil(whole_list_byte_size/max_bytes_per_file)
        num_configs_per_file = math.ceil(len(pickable_iterable)/num_files)
        config_idx_list = list(range(0, len(pickable_iterable)))
        config_idx_splits = [config_idx_list[i:i + num_configs_per_file] \
                     for range(0, len(config_idx_list), num_configs_per_file)]
        for config_idx_split in config_idx_splits:
            iterable_to_write = None
            if isinstance(pickable_iterable, list):
                iterable_to_write = pickable_iterable[config_idx_split]
            elif isinstance(pickable_iterable, dict):
                iterable_to_write = { config_idx : pickable_iterable[config_idx] \
                                      for config_idx in config_idx_split}

            filename = os.path.join(filetype, "config_idx_{}_to_{}.{}".format(
                  config_idx_split[0], config_idx_split[-1], filetype))
            zip_file_handle.writestr( filename, pickle.dumps(iterable_to_write))

    def dump(self, filename, dump_configs=True):
        if isinstance(self.__data_frame, pd.DataFrame):
            self.__data_frame = None
        if not dump_configs:
            self.__benchmark_configs = None
        with open(filename, 'wb') as handle:
            pickle.dump(self, handle, protocol=pickle.HIGHEST_PROTOCOL)
        logging.info("Saved BenchmarkResult to {}".format(
            os.path.abspath(filename)))

    def extend(self, benchmark_result):
        new_idxs = benchmark_result.get_benchmark_config_indices()
        this_idxs = self.get_benchmark_config_indices()
        overlap = set(new_idxs) & set(this_idxs)
        if len(overlap) != 0:
            raise ValueError("Overlapping config indices. No extension possible.")
        self.__result_dict.extend(benchmark_result.get_result_dict())
        self.__benchmark_configs.extend(benchmark_result.get_benchmark_configs())
        self.__data_frame = None
        self.__histories.update(benchmark_result.get_histories())