# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import unittest
import os
import random
import pandas as pd

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

def random_history_data(history_num, hist_size, multiply=1, offset=0):
    histories = {i : bytearray(os.urandom(hist_size)) \
             for i in range(offset, history_num*multiply+ offset, multiply)}
    return histories

class TestConfig:
    def __init__(self, idx, conf_size):
        self.bytes = bytearray(os.urandom(conf_size))
        self.config_idx = idx
    def __eq__(self, other):
        return self.bytes == other.bytes and \
          self.config_idx == other.config_idx
def random_benchmark_conf_data(num_confs, conf_size, hist_size=100, multiply=1, offset=0):
    result_data = random_result_data(num_confs)
    hist_data = random_history_data(num_confs, hist_size, multiply, offset)
    for idx, result in enumerate(result_data):
      result["config_idx"] = idx + offset
    confs = [ TestConfig(i+ offset, conf_size )for i in range(0, num_confs)]
    return confs, result_data, hist_data

class DatabaseRunnerTests(unittest.TestCase):

    def test_dump_and_load_results(self):
        result_data = random_result_data(size = 10)
        br = BenchmarkResult(result_dict=result_data)
        br.dump("./results")
        br_loaded = BenchmarkResult.load("./results")
        loaded_dict = br_loaded.get_result_dict()
        self.assertEqual(result_data, loaded_dict)

    def test_dump_and_load_histories_many(self):
        result_num = 100
        result_data = random_result_data(size = result_num)
        histories = random_history_data(result_num, 2000000, multiply=3)
        br = BenchmarkResult(result_dict=result_data, histories=histories)
        br.dump("./results_with_history", dump_histories=True, max_mb_per_file = 5)
        br_loaded = BenchmarkResult.load("./results_with_history")
        br_loaded.load_histories(config_idx_list = list(histories.keys()))
        loaded_histories = br_loaded.get_histories()
        self.assertEqual(histories, loaded_histories)
        loaded_dict = br_loaded.get_result_dict()
        self.assertEqual(result_data, loaded_dict)

    def test_dump_and_load_histories_one(self):
        result_num = 2
        result_data = random_result_data(size = result_num)
        histories = random_history_data(result_num, 20)
        br = BenchmarkResult(result_dict=result_data, histories=histories)
        br.dump("./results_with_history", dump_histories=True, max_mb_per_file = 2)
        br_loaded = BenchmarkResult.load("./results_with_history")
        br_loaded.load_histories(config_idx_list = list(histories.keys()))
        loaded_histories = br_loaded.get_histories()
        self.assertEqual(histories, loaded_histories)
        loaded_dict = br_loaded.get_result_dict()
        self.assertEqual(result_data, loaded_dict)

    def test_dump_and_load_benchmark_configs(self):
        result_num = 100
        confs, result_data, _ = random_benchmark_conf_data(result_num, 2000000)
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
        confs, result_data, histories = random_benchmark_conf_data(result_num, 2000000, hist_size=1500000)
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

    def test_extend_from_file(self):
        try:
          os.remove("./br1")
          os.remove("./br2")
          os.remove("./br3")
        except:
          pass
        result_num = 100
        confs, result_data, histories1 = random_benchmark_conf_data(result_num, 2000000, hist_size=1500000, offset=0)
        br1 = BenchmarkResult(result_dict=result_data,
          benchmark_configs=confs, histories=histories1)
        br1.dump("./br1", dump_histories=True, dump_configs=True)
        br1_df = br1.get_data_frame().copy()

        result_num = 30
        confs2, result_data2, histories2 = random_benchmark_conf_data(result_num, 2000000, hist_size=1500000, offset=200)
        br2 = BenchmarkResult(result_dict=result_data2,
          benchmark_configs=confs2, histories=histories2)
        br2.dump(filename="./br2", dump_histories=True, dump_configs=True)

        result_num = 10
        confs3, result_data3, histories3 = random_benchmark_conf_data(result_num, 2000000, hist_size=1500000, offset=400)
        br3 = BenchmarkResult(result_dict=result_data3,
          benchmark_configs=confs3, histories=histories3)
        br3.dump(filename="./br3", dump_histories=True, dump_configs=True)

        br1.extend(benchmark_result=br2, file_level=True)
        br1.extend(benchmark_result=br3, file_level=True)

        br_loaded = BenchmarkResult.load("./br1", load_histories=True, load_configs=True)
        df_desired = br1_df
        df_desired = pd.concat([df_desired, br2.get_data_frame()])
        df_desired = pd.concat([df_desired, br3.get_data_frame()])
        self.assertEqual(len(br_loaded.get_data_frame().index), len(df_desired.index))

        extended_confs = br_loaded.get_benchmark_configs()
        self.assertEqual(len(extended_confs), 140)
        extended_histories = br_loaded.get_histories()
        self.assertEqual(len(extended_histories), 140)
        extended_histories = histories1
        extended_histories.update(histories2)
        extended_histories.update(histories3)
        for bc in extended_confs:
            self.assertEqual(br_loaded.get_history(bc.config_idx), extended_histories[bc.config_idx])
        

if __name__ == '__main__':
    unittest.main()


