# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import pickle
import pandas as pd
import logging
logging.getLogger().setLevel(logging.INFO)

from modules.runtime.commons.parameters import ParameterServer
from bark.world.evaluation import * 

# contains information for a single benchmark run
class BenchmarkConfig:
  def __init__(self, config_idx, behavior, behavior_name,
    scenario, scenario_idx, scenario_set_name):
    self.config_idx = config_idx
    self.behavior = behavior
    self.behavior_name = behavior_name
    self.scenario = scenario
    self.scenario_idx = scenario_idx
    self.scenario_set_name = scenario_set_name

# result of benchmark run
class BenchmarkResult:
  def __init__(self, result_dict, benchmark_configs, **kwargs):
    self.__result_dict = result_dict
    self.__benchmark_configs = benchmark_configs
    self.__data_frame = None

  def get_data_frame(self):
      if not self.__data_frame:
          self.__data_frame = pd.DataFrame(self.__result_dict)
      return self.__data_frame

  def get_result_dict(self):
      return self.__result_dict

  def get_benchmark_configs(self):
      return self.__benchmark_configs

  def get_eval_config(self, config_idx):
      if not self.__benchmark_configs:
          self._sort_bench_confs()
      bench_conf = self.__benchmark_configs[config_idx]
      assert(bench_conf.config_idx == config_idx)
      return bench_conf

  def _sort_bench_confs(self):
      def sort_key(bench_conf):
          return bench_conf.config_idx
      self.__benchmark_configs.sort(key=sort_key)

  @staticmethod 
  def load(filename):
      with open(filename, 'rb') as handle:
          dmp = pickle.load(handle)
      return dmp

  def dump(self, filename):
      if self.data_frame:
          self.data_frame = None
      with open(filename, 'wb') as handle:
          pickle.dump(self, handle, protocol=pickle.HIGHEST_PROTOCOL)
      logging.info("Saved BenchmarkResult to {}".format(
        filename))

class BenchmarkRunner:
    def __init__(self,
               benchmark_database=None,
               evaluators=None,
               terminal_when=None,
               behaviors=None,
               num_scenarios=None,
               benchmark_configs=None):

        self.benchmark_database = benchmark_database
        self.evaluators = evaluators or {}
        self.terminal_when = terminal_when or []
        self.behaviors = behaviors or {}
        self.benchmark_configs = benchmark_configs or \
               self._create_configurations(num_scenarios)

    def _create_configurations(self, num_scenarios=None):
      benchmark_configs = []
      for behavior_name, behavior_bark in self.behaviors.items():
            # run over all scenario generators from benchmark database
            for scenario_generator, scenario_set_name in self.benchmark_database:
                  for scenario, scenario_idx in scenario_generator:
                    if num_scenarios and scenario_idx >= num_scenarios:
                      break
                    benchmark_config = \
                    BenchmarkConfig(
                      len(benchmark_configs),
                      behavior_bark,
                      behavior_name,
                      scenario,
                      scenario_idx,
                      scenario_set_name
                    )
                    benchmark_configs.append(benchmark_config)
      return benchmark_configs

    def run(self):
      results = []
      for idx, bmark_conf in enumerate(self.benchmark_configs):
        logging.info("Running config idx {}/{}: Scenario {} of set \"{}\" for behavior \"{}\"".format(
            idx, len(self.benchmark_configs)-1, bmark_conf.scenario_idx,
            bmark_conf.scenario_set_name, bmark_conf.behavior_name))
        result_dict = self._run_benchmark_config(bmark_conf)
        results.append(result_dict)
      return BenchmarkResult(results, self.benchmark_configs)
                    
    def _run_benchmark_config(self, benchmark_config):
        scenario = benchmark_config.scenario
        behavior = benchmark_config.behavior
        parameter_server = ParameterServer(json=scenario._json_params)
        world = scenario.get_world_state()
        old_behavior = world.agents[scenario._eval_agent_ids[0]].behavior_model
        world.agents[scenario._eval_agent_ids[0]].behavior_model = behavior
        self._reset_evaluators(world, scenario._eval_agent_ids)

        step_time = parameter_server["Simulation"]["StepTime", "", 0.2]
        terminal = False
        step = 0
        terminal_why = None
        while not terminal:
            evaluation_dict = self._get_evalution_dict(world)
            terminal, terminal_why = self._is_terminal(evaluation_dict)
            world.Step(step_time) 
            step += 1

        # maintain state to avoid complicated deserialization of eg mcts in multiprocessing case 
        world.agents[scenario._eval_agent_ids[0]].behavior_model = old_behavior
        
        dct = {"scen_set": benchmark_config.scenario_set_name,
              "scen_idx" : benchmark_config.scenario_idx,
              "step": step,
              "behavior" : benchmark_config.behavior_name,
              **evaluation_dict,
              "Terminal": terminal_why}

        return dct

    def _reset_evaluators(self, world, eval_agent_ids):
        for evaluator_name, evaluator_type in self.evaluators.items():
            evaluator_bark = None
            try:
                evaluator_bark = eval("{}(eval_agent_ids[0])".format(evaluator_type))
            except:
                evaluator_bark = eval("{}()".format(evaluator_type))
            world.AddEvaluator(evaluator_name, evaluator_bark)

    def _get_evalution_dict(self, world):
        return world.Evaluate()

    def _is_terminal(self, evaluation_dict):
        terminal = False
        terminal_why = []
        for evaluator_name, function in self.terminal_when.items():
          if function(evaluation_dict[evaluator_name]):
            terminal = True
            terminal_why.append(evaluator_name)
        return terminal, terminal_why