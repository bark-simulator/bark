# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT


import ray
import logging
import psutil
logging.getLogger().setLevel(logging.INFO)

from modules.benchmark.benchmark_runner import BenchmarkRunner, BenchmarkResult

# implement a parallelized version of benchmark running based on ray

# actor class running on a single core
@ray.remote
class _BenchmarkRunnerActor(BenchmarkRunner):
  def __init__(self, benchmark_configs, core_idx):
        super().__init__(benchmark_configs=benchmark_configs)
        self.core_idx = core_idx

# runner spawning actors and distributing benchmark configs
class BenchmarkRunnerMP(BenchmarkRunner):
    def __init__(self, benchmark_database=None,
               evaluators=None,
               terminal_when=None,
               behaviors=None,
               num_scenarios=None,
               benchmark_configs=None, num_cpus=None):
        super().__init__(benchmark_database=benchmark_database,
                          evaluators=evaluators, terminal_when=terminal_when,
                          behaviors=behaviors, benchmark_configs=benchmark_configs,
                          num_scenarios=num_scenarios)
        num_cpus_available = psutil.cpu_count(logical=True)
        if num_cpus and num_cpus <= num_cpus_available:
            pass
        else:
            num_cpus = num_cpus_available
        ray.init(num_cpus=num_cpus)
        self.benchmark_config_split = [self.benchmark_configs[i::num_cpus] for i in range(0, num_cpus)]
        self.actors = [_BenchmarkRunnerActor.remote(self.benchmark_config_split[i], i) for i in range(num_cpus) ]

    def run(self):
        results_tmp = ray.get([actor.run.remote() for actor in self.actors])
        result_dict = []
        benchmark_configs = []
        for result_tmp in results_tmp:
            result_dict.extend(result_tmp.get_result_dict())
            benchmark_configs.extend(result_tmp.get_benchmark_configs())
        ray.shutdown()
        return BenchmarkResult(result_dict, eval_configs)