# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import unittest
import os
import random

try:
    import debug_settings
except:
    pass


from bark.benchmark.benchmark_result import BenchmarkResult

def random_result_data(size):
    columns = ['value1', 'value2', 'value3', 'value4']
    results = [ {column : random.randint(0, 100) \
             for column in columns}  for _ in range(0, size)]
    return results

def random_history_data(history_num, hist_size):
    histories = {i : bytearray(os.urandom(hist_size)) \
             for i in range(0, history_num)}
    return histories

class TestConfig:
    def __init__(self, idx, conf_size):
        self.bytes = bytearray(os.urandom(conf_size))
        self.config_idx = idx
    def __eq__(self, other):
        return self.bytes == other.bytes and \
          self.config_idx == other.config_idx
def random_benchmark_conf_data(num_confs, conf_size):
    confs = [ TestConfig(i, conf_size)for i in range(0, num_confs)]
    return confs

class DatabaseRunnerTests(unittest.TestCase):
    def test_dump_and_load_results(self):
        result_data = random_result_data(size = 10)
        br = BenchmarkResult(result_dict=result_data)
        br.dump("./results")
        br_loaded = BenchmarkResult.load("./results")
        loaded_dict = br_loaded.get_result_dict()
        self.assertEqual(result_data, loaded_dict)

    def test_dump_and_load_histories(self):
        result_num = 100
        result_data = random_result_data(size = result_num)
        histories = random_history_data(result_num, 2000000)
        br = BenchmarkResult(result_dict=result_data, histories=histories)
        br.dump("./results_with_history", dump_histories=True, max_mb_per_file = 5)
        br_loaded = BenchmarkResult.load("./results_with_history")
        br_loaded.load_histories(config_idx_list = list(range(0, result_num)))
        loaded_histories = br_loaded.get_histories()
        self.assertEqual(histories, loaded_histories)
        loaded_dict = br_loaded.get_result_dict()
        self.assertEqual(result_data, loaded_dict)

    def test_dump_and_load_benchmark_configs(self):
        result_num = 100
        result_data = random_result_data(size = result_num)
        confs = random_benchmark_conf_data(result_num, 2000000)
        br = BenchmarkResult(result_dict=result_data, benchmark_configs=confs)
        br.dump("./results_with_confs", dump_configs=True, max_mb_per_file = 5)
        br_loaded = BenchmarkResult.load("./results_with_confs")
        br_loaded.load_benchmark_configs(config_idx_list = list(range(0, result_num)))
        loaded_confs = br_loaded.get_benchmark_configs()
        self.assertEqual(confs, loaded_confs)
        loaded_dict = br_loaded.get_result_dict()
        self.assertEqual(br.get_result_dict(), loaded_dict)

    def test_dump_and_partial_load(self):
        result_num = 100
        result_data = random_result_data(size = result_num)
        confs = random_benchmark_conf_data(result_num, 2000000)
        histories = random_history_data(result_num, 1500000)
        br = BenchmarkResult(result_dict=result_data,
          benchmark_configs=confs, histories=histories)
        br.dump("./results_all", dump_configs=True, dump_histories=True, max_mb_per_file = 5)
        br_loaded = BenchmarkResult.load("./results_all")
        loaded_dict = br_loaded.get_result_dict()
        self.assertEqual(br.get_result_dict(), loaded_dict)

        loaded_configs_idx = list(range(10, 20))
        processed_files = br_loaded.load_benchmark_configs(config_idx_list = loaded_configs_idx)
        loaded_confs = br_loaded.get_benchmark_configs()
        self.assertEqual(len(loaded_confs), 10)
        # 2mb per conf, max 5 mb per file -> 2 confs per file -> 10/2 = 5files
        self.assertEqual(len(processed_files), 5) 
        for conf_idx in loaded_configs_idx:
            self.assertEqual(br_loaded.get_benchmark_config(conf_idx), confs[conf_idx])

        loaded_configs_idx = list(range(10, 27))
        processed_files = br_loaded.load_histories(config_idx_list = loaded_configs_idx)
        loaded_histories = br_loaded.get_histories()
        self.assertEqual(len(loaded_histories), 18) # one more as specified since it was in the last file
        # 1.5mb per history, max 5 mb per file -> 3 confs per file -> 17/3 = 6files
        self.assertEqual(len(processed_files), 6)
        for conf_idx in loaded_configs_idx:
            self.assertEqual(br_loaded.get_history(conf_idx), histories[conf_idx])


if __name__ == '__main__':
    unittest.main()


