# Copyright (c) 2020 Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import pickle 
import ray
import logging
import psutil
import inspect

import bark.core.world.evaluation
logging.getLogger().setLevel(logging.INFO)

from bark.runtime.scenario.scenario import Scenario
from bark.benchmark.benchmark_runner import BenchmarkRunner, BenchmarkResult, BenchmarkConfig

# implement a parallelized version of benchmark running based on ray

# custom serialization as cloudpickling fails
def serialize_benchmark_config(bc):
  return pickle.dumps(bc)

def deserialize_benchmark_config(bc):
  return pickle.loads(bc)

def serialize_scenario(sc):
  return pickle.dumps(sc)

def deserialize_scenario(sc):
  return pickle.loads(sc)


# actor class running on a single core
@ray.remote
class _BenchmarkRunnerActor(BenchmarkRunner):
    def __init__(self, evaluators, terminal_when, benchmark_configs, logger_name, log_eval_avg_every):
          super().__init__(evaluators=evaluators, 
                            terminal_when=terminal_when,
                            benchmark_configs=benchmark_configs,
                            logger_name=logger_name,
                            log_eval_avg_every=log_eval_avg_every)

# runner spawning actors and distributing benchmark configs
class BenchmarkRunnerMP(BenchmarkRunner):
    def __init__(self, benchmark_database=None,
               evaluators=None,
               terminal_when=None,
               behaviors=None,
               num_scenarios=None,
               benchmark_configs=None,
               log_eval_avg_every=None,
               num_cpus=None,
               memory_total=None):
        super().__init__(benchmark_database=benchmark_database,
                          evaluators=evaluators, terminal_when=terminal_when,
                          behaviors=behaviors, num_scenarios=num_scenarios,
                          benchmark_configs=benchmark_configs)
        num_cpus_available = psutil.cpu_count(logical=True)
        if num_cpus and num_cpus <= num_cpus_available:
          pass
        else:
          num_cpus = num_cpus_available
        
        mem = psutil.virtual_memory()
        memory_available = mem.available
        if memory_total and memory_total <= memory_available:
          pass
        else:
          memory_total = memory_available

        ray.init(num_cpus=num_cpus, memory=memory_total*0.3, object_store_memory=memory_total*0.7) # we split memory between workers (30%) and objects (70%)
        
        ray.register_custom_serializer(
          BenchmarkConfig, serializer=serialize_benchmark_config,
          deserializer=deserialize_benchmark_config)
        ray.register_custom_serializer(
          Scenario, serializer=serialize_scenario,
          deserializer=deserialize_scenario)
        self.benchmark_config_split = [self.benchmark_configs[i::num_cpus] for i in range(0, num_cpus)]
        self.actors = [_BenchmarkRunnerActor.remote(evaluators=evaluators,
                                                    terminal_when=terminal_when,
                                                    benchmark_configs=self.benchmark_config_split[i],
                                                    logger_name="BenchmarkingActor{}".format(i),
                                                    log_eval_avg_every=log_eval_avg_every) for i in range(num_cpus) ]

    def run(self, viewer = None, maintain_history = False):
        results_tmp = ray.get([actor.run.remote(viewer, maintain_history) for actor in self.actors])
        result_dict = []
        benchmark_configs = []
        histories = {}
        for result_tmp in results_tmp:
            result_dict.extend(result_tmp.get_result_dict())
            benchmark_configs.extend(result_tmp.get_benchmark_configs())
            histories.update(result_tmp.get_histories())
        return BenchmarkResult(result_dict, benchmark_configs, histories=histories)

    def __del__(self):
       ray.shutdown()