# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import pickle 
import time
import ray
import logging
import psutil
import inspect
import os
import sys

from bark.core.commons import GLogInit
logging.getLogger().setLevel(logging.INFO)

from bark.runtime.scenario.scenario import Scenario
from bark.benchmark.benchmark_result import BenchmarkResult, BenchmarkConfig
from bark.benchmark.benchmark_runner import BenchmarkRunner 

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
    def __init__(self, serialized_evaluators, terminal_when, benchmark_configs, logger_name, log_eval_avg_every, checkpoint_dir, actor_id, glog_init_settings=None):
        evaluators = pickle.loads(serialized_evaluators) # unpickle
        super().__init__(evaluators=evaluators, 
                        terminal_when=terminal_when,
                        benchmark_configs=benchmark_configs,
                        logger_name=logger_name,
                        log_eval_avg_every=log_eval_avg_every,
                        checkpoint_dir=checkpoint_dir,
                        merge_existing=False)
        glog_init_settings = glog_init_settings or {}
        log_folder_name = glog_init_settings.pop("log_folder", "logs")
        log_folder = os.path.abspath(os.path.join(os.getcwd(), log_folder_name))
        if not os.path.exists(log_folder):
          os.makedirs(log_folder)
        self.logger.info("Logging into: {}".format(log_folder))
        vlevel = glog_init_settings.pop("vlevel", 1)
        log_stderr = glog_init_settings.pop("log_stderr", False)
        #GLogInit(sys.argv[0], log_folder, vlevel, log_stderr)
        self.actor_id = actor_id

    def get_checkpoint_file_name(self):
        return "benchmark_runner_actor{}.ckpnt".format(self.actor_id)


# runner spawning actors and distributing benchmark configs
class BenchmarkRunnerMP(BenchmarkRunner):
    def __init__(self, benchmark_database=None,
               evaluators=None,
               terminal_when=None,
               behaviors=None,
               behavior_configs=None,
               num_scenarios=None,
               scenario_generation=None,
               benchmark_configs=None,
               log_eval_avg_every=None,
               glog_init_settings=None,
               checkpoint_dir=None,
               merge_existing=False,
               num_cpus=None,
               memory_total=None,
               ip_head=None,
               redis_password=None):
        super().__init__(benchmark_database=benchmark_database,
                          evaluators=evaluators, terminal_when=terminal_when,
                          behaviors=behaviors, behavior_configs=behavior_configs, num_scenarios=num_scenarios,
                          benchmark_configs=benchmark_configs, scenario_generation=scenario_generation,
                          checkpoint_dir=checkpoint_dir, merge_existing=merge_existing)
        num_cpus_available = psutil.cpu_count(logical=True)

        if ip_head and redis_password:
          ray.init(address=ip_head, redis_password=redis_password)
        else:
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
          
          ray.init(num_cpus=num_cpus, memory=memory_total*0.3, object_store_memory=memory_total*0.7, \
             _internal_config='{"initial_reconstruction_timeout_milliseconds": 100000}') # we split memory between workers (30%) and objects (70%)
        
        serialized_evaluators = pickle.dumps(self.evaluators)
        ray.register_custom_serializer(
          BenchmarkConfig, serializer=serialize_benchmark_config,
          deserializer=deserialize_benchmark_config)
        ray.register_custom_serializer(
          Scenario, serializer=serialize_scenario,
          deserializer=deserialize_scenario)
        self.benchmark_config_split = [self.configs_to_run[i::num_cpus] for i in range(0, num_cpus)]
        self.actors = [_BenchmarkRunnerActor.remote(serialized_evaluators=serialized_evaluators,
                                                    terminal_when=terminal_when,
                                                    benchmark_configs=self.benchmark_config_split[i],
                                                    logger_name="BenchmarkingActor{}".format(i),
                                                    log_eval_avg_every=log_eval_avg_every,
                                                    checkpoint_dir=checkpoint_dir,
                                                    actor_id=i,
                                                    glog_init_settings=glog_init_settings) for i in range(num_cpus) ]

    def run(self, viewer = None, maintain_history = False, checkpoint_every=None):
        results_tmp = ray.get([actor.run.remote(viewer, maintain_history, checkpoint_every) for actor in self.actors])
        intermediate_result = BenchmarkResult(file_name= \
                os.path.abspath(os.path.join(self.checkpoint_dir, self.get_checkpoint_file_name())))
        for result_tmp in results_tmp:
            logging.info("Result file: {}".format(result_tmp.get_file_name()))
            intermediate_result.extend(result_tmp, file_level=True) 
        intermediate_result.extend(self.existing_benchmark_result, file_level=True)
        return intermediate_result

    def __del__(self):
       ray.shutdown()