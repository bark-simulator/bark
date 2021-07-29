# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import os
import pickle
import pandas as pd
import logging
import copy
import time
import glob
import numpy as np
import importlib

logging.basicConfig()
logging.getLogger().setLevel(logging.INFO)

from bark.runtime.commons.parameters import ParameterServer
from bark.runtime.scenario.scenario import Scenario
from bark.benchmark.benchmark_result import BenchmarkResult, BenchmarkConfig, BehaviorConfig
from bark.core.world.evaluation import *

try:
  from bark.core.world.evaluation.ltl import *
except Exception as e:
  logging.warning("LTL evaluators not loaded: {}".format(e))



class EvaluationConfig:
  __EVALUATION_MODULES = []

  def __init__(self, evaluators = None):
    self.evaluators_scenario_specific = {}
    self.evaluators_default = evaluators or None
    self.import_module_names = EvaluationConfig.__EVALUATION_MODULES

  def AddEvaluatorConfig(self, evaluator_config, scenario_type=None):
    if scenario_type:
      self.evaluators_scenario_specific[scenario_type] = evaluator_config
    else:
      self.evaluators_default = evaluator_config

  def GetEvaluationCriteria(self):
    evaluation_criteria = set()
    if self.evaluators_default:
      evaluation_criteria.update(self.evaluators_default.keys())
    for _, evaluation_config in self.evaluators_scenario_specific.items():
      evaluation_criteria.update(evaluation_config.keys())
    return list(evaluation_criteria)

  @staticmethod
  def AddEvaluationModule(module_name):
    EvaluationConfig.__EVALUATION_MODULES.append(module_name)
  
  def GetEvaluationModuleNames(self):
    return self.import_module_names

  def _GetScenarioEvaluators(self, scenario_set_name):
    if len(self.evaluators_scenario_specific) > 1:
        evaluator_names = list(self.evaluators_scenario_specific.keys())
        matching = [name for name in evaluator_names if name in scenario_set_name]
        if len(matching) == 0:
          return self.evaluators_default 
        if len(matching) > 1:
          raise ValueError("Invalid matching of evaluation configs {} to scenario set {}".format(matching, scenario_set_name))
        else:
          return self.evaluators_scenario_specific[matching[0]]
    else:
      return self.evaluators_default

  def CreateInitializedEvaluators(self, eval_agent_ids, scenario_set_name):
    current_evaluators = self._GetScenarioEvaluators(scenario_set_name)
    evaluators_initialized = {}
    for evaluator_name, evaluator_params in current_evaluators.items():
      evaluator_bark = None
      if isinstance(evaluator_params, str):
          try:
              evaluator_bark = eval("{}(eval_agent_ids[0])".format(evaluator_params))
          except:
              evaluator_bark = eval("{}()".format(evaluator_params)) 
      elif isinstance(evaluator_params, dict):
          try:
            evaluator_bark = eval(
              "{}(agent_id=eval_agent_ids[0], **evaluator_params['params'])".format(evaluator_params["type"]))
          except:
            try:
              evaluator_bark = eval(
                "{}(evaluator_params['params'], eval_agent_ids[0])".format(evaluator_params["type"]))
            except NameError:
              for module_name in self.GetEvaluationModuleNames():
                try:
                  module = importlib.import_module(module_name)
                  eval_type = getattr(module, evaluator_params["type"])
                  evaluator_bark = eval("eval_type(evaluator_params['params'], eval_agent_ids[0])")
                except:
                  pass
              if not evaluator_bark:
                raise ValueError("Invalid evaluation spec.")
      elif issubclass(type(evaluator_params), BaseEvaluator):
          evaluator_bark = copy.deepcopy(evaluator_params)
          evaluator_bark.SetAgentId(eval_agent_ids[0])
      else:
          raise ValueError("Invalid evaluation spec.")
      evaluators_initialized[evaluator_name] = evaluator_bark
    return evaluators_initialized


class BenchmarkRunner:
    def __init__(self,
                 benchmark_database=None,
                 evaluators=None,
                 terminal_when=None,
                 behaviors=None,
                 behavior_configs=None,
                 num_scenarios=None,
                 benchmark_configs=None,
                 scenario_generation=None,
                 logger_name=None,
                 log_eval_avg_every=None,
                 checkpoint_dir=None,
                 merge_existing=False,
                 deepcopy=True):

        self.benchmark_database = benchmark_database
        self.scenario_generation = scenario_generation
        self.evaluators = evaluators if isinstance(evaluators, EvaluationConfig) else EvaluationConfig(evaluators)
        self.terminal_when = terminal_when or []
        if behaviors:
          self.behavior_configs = BehaviorConfig.configs_from_dict(behaviors)
        else:
          self.behavior_configs = behavior_configs or {}
        if benchmark_configs:
          self.benchmark_configs = benchmark_configs
        elif benchmark_database:
          self.benchmark_configs = \
                                 self._create_configurations_from_database(num_scenarios)
        elif scenario_generation:
          self.benchmark_configs = \
                                  self._create_configurations_from_scenario_generation(num_scenarios)
        else:
          self.benchmark_configs = [] # to be compatible when benchmark runner is used in ray actor

        self.logger = logging.getLogger(logger_name or "BenchmarkRunner")
        self.logger.setLevel(logging.DEBUG)
        self.logger.info("Total number of {} configs to run".format(len(self.benchmark_configs)))
        self.configs_to_run = self.benchmark_configs
        self._deepcopy = deepcopy
        self.checkpoint_dir = checkpoint_dir or "checkpoints"
        self.existing_benchmark_result = BenchmarkResult(file_name= \
                    BenchmarkRunner.get_merged_result_filename(self.checkpoint_dir))
        if not os.path.exists(self.checkpoint_dir):
            os.makedirs(self.checkpoint_dir)

        if merge_existing:
            self.existing_benchmark_result = \
                BenchmarkRunner.merge_checkpoint_benchmark_results(checkpoint_dir)
            self.logger.info("Merged {} processed configs in folder {}". \
                format(len(self.existing_benchmark_result.get_benchmark_configs()), checkpoint_dir))
            self.configs_to_run = self.get_configs_to_run(self.benchmark_configs, \
                                                            self.existing_benchmark_result)
            self.logger.info("Remaining  number of {} configs to run".format(len(self.configs_to_run)))

        self.exceptions_caught = []
        self.log_eval_avg_every = log_eval_avg_every

    def get_checkpoint_file_name(self):
      return "benchmark_runner.ckpnt"

    def clear_checkpoint_dir(self):
      files = glob.glob(os.path.join(self.checkpoint_dir, "*.ckpnt"))
      for f in files:
          os.remove(f)

    @staticmethod
    def get_merged_result_filename(checkpoint_dir):
        return os.path.abspath(os.path.join(checkpoint_dir,"merged_results.ckpnt"))
    
    @staticmethod
    def merge_checkpoint_benchmark_results(checkpoint_dir):
        checkpoint_files = glob.glob(os.path.join(checkpoint_dir, "**/*.ckpnt"), recursive=True)
        merged_result_filename = BenchmarkRunner.get_merged_result_filename(checkpoint_dir)
        if os.path.exists(merged_result_filename):
          merged_result = BenchmarkResult.load_results(filename=merged_result_filename)
        else:
          merged_result = BenchmarkResult(file_name=merged_result_filename)
        # merge all checkpoints with new results
        for checkpoint_file in checkpoint_files:
          loaded_result = BenchmarkResult.load(os.path.abspath(checkpoint_file))
          merged_result.extend(loaded_result, file_level=True)
          logging.info("Extending with checkpoint {}".format(checkpoint_file))

        # delete checkpoints
        for checkpoint_file in checkpoint_files:
          if "merged_result" in checkpoint_file:
            continue
          os.remove(checkpoint_file)
          logging.info("Removed old checkpoint file {}".format(checkpoint_file))
        return merged_result

    @staticmethod
    def get_configs_to_run(benchmark_configs, existing_benchmark_result):
        existing_inds = existing_benchmark_result.get_benchmark_config_indices()
        required_inds = BenchmarkResult(benchmark_configs=benchmark_configs).get_benchmark_config_indices()
        missing_inds = list(set(required_inds) - set(existing_inds))

        filtered_configs = filter(lambda bc : bc.config_idx in missing_inds, benchmark_configs)
        return list(filtered_configs)

    def _create_configurations_from_database(self, num_scenarios=None):
        benchmark_configs = []
        for behavior_config in self.behavior_configs:
            # run over all scenario generators from benchmark database
            for scenario_generator, scenario_set_name, scenario_set_param_desc in self.benchmark_database:
                for scenario, scenario_idx in scenario_generator:
                    if num_scenarios and scenario_idx >= num_scenarios:
                        break
                      
                    # only relevant for scenarios from dataset
                    dataset_scenario_desc = scenario.GetDatasetScenarioDescription()
                    temp_scenario_set_param_desc = scenario_set_param_desc.copy()
                    temp_scenario_set_param_desc.update(dataset_scenario_desc)
                      
                    benchmark_config = \
                        BenchmarkConfig(
                            len(benchmark_configs),
                            behavior_config,
                            scenario,
                            scenario_idx,
                            scenario_set_name,
                            temp_scenario_set_param_desc
                        )
                    benchmark_configs.append(benchmark_config)
        return benchmark_configs
    
    def _create_configurations_from_scenario_generation(self, num_scenarios):
        benchmark_configs = []
        for behavior_config in self.behavior_configs:
          for scenario, scenario_idx in self.scenario_generation:
            if num_scenarios and scenario_idx >= num_scenarios:
              break
            benchmark_config = \
                        BenchmarkConfig(
                            len(benchmark_configs),
                            behavior_config,
                            scenario,
                            scenario_idx,
                            type(self.scenario_generation).__name__,
                            {}
                        )
            benchmark_configs.append(benchmark_config)
        return benchmark_configs

    def run(self, viewer=None, maintain_history=False, checkpoint_every=None):
        last_results = []
        last_histories = {}
        last_run_configs = []
        results = []
        checkpoint_file = os.path.abspath(os.path.join(self.checkpoint_dir, self.get_checkpoint_file_name()))
        last_result_file = os.path.abspath(os.path.join(self.checkpoint_dir, "tmp_{}".format(self.get_checkpoint_file_name())))
        checkpoint_result = BenchmarkResult(file_name=checkpoint_file)
        for idx, bmark_conf in enumerate(self.configs_to_run):
            self.logger.info("Running config idx {} being {}/{}: Scenario {} of set \"{}\" for behavior \"{}\"".format(
                bmark_conf.config_idx, idx, len(self.benchmark_configs) - 1, bmark_conf.scenario_idx,
                bmark_conf.scenario_set_name, bmark_conf.behavior_config.behavior_name))
            bmark_conf = copy.deepcopy(bmark_conf) if self._deepcopy else bmark_conf
            result_dict, scenario_history = self._run_benchmark_config(bmark_conf, viewer,
                                                                       maintain_history)
            results.append(result_dict)
            last_results.append(result_dict)
            last_histories[bmark_conf.config_idx] = scenario_history
            last_run_configs.append(bmark_conf)
            if self.log_eval_avg_every and (idx + 1) % self.log_eval_avg_every == 0:
                self._log_eval_average(results, self.configs_to_run)

            if checkpoint_every and (idx+1) % checkpoint_every == 0:
                # append results since last checkpoint
                last_benchmark_result = BenchmarkResult(result_dict=last_results, file_name = last_result_file, \
                         benchmark_configs=last_run_configs, histories=last_histories)
                last_benchmark_result.dump(last_result_file, dump_configs=True, dump_histories=maintain_history, append=False)
                checkpoint_result.extend(benchmark_result=last_benchmark_result, file_level=True)
                self.logger.info("Extended checkpoint {} with last result.".format(checkpoint_file))
                last_histories.clear()
                last_run_configs.clear()
                last_results.clear()
        # append results of last run
        last_benchmark_result = BenchmarkResult(result_dict=last_results, file_name = last_result_file, \
                         benchmark_configs=last_run_configs, histories=last_histories)
        last_benchmark_result.dump(last_result_file, dump_configs=True, dump_histories=maintain_history, append=False)
        checkpoint_result.extend(benchmark_result=last_benchmark_result, file_level=True)
        os.remove(last_result_file)
        self.logger.info("Extended checkpoint {} with final result.".format(checkpoint_file))
        checkpoint_result.extend(benchmark_result=self.existing_benchmark_result, file_level=True)
        return checkpoint_result

    def run_benchmark_config(self, config_idx, **kwargs):
        for idx, bmark_conf in enumerate(self.benchmark_configs):
            if bmark_conf.config_idx == config_idx:
                bmark_conf = copy.deepcopy(bmark_conf) if self._deepcopy else bmark_conf
                result_dict, scenario_history = self._run_benchmark_config(bmark_conf, **kwargs)
                return BenchmarkResult(result_dict, [bmark_conf], histories={config_idx : scenario_history})
        self.logger.error("Config idx {} not found in benchmark configs. Skipping...".format(config_idx))
        return

    def _run_benchmark_config(self, benchmark_config, viewer=None, maintain_history=False):
        scenario = benchmark_config.scenario
        scenario_set_name = benchmark_config.scenario_set_name
        behavior = benchmark_config.behavior_config.behavior
        parameter_server = ParameterServer(json=scenario._json_params)
        scenario_history = []
        step = 0
        if viewer:
          viewer.initialize_params(parameter_server)
        try:
            world = scenario.GetWorldState()
        except Exception as e:
            self.logger.error("For config-idx {}, Exception thrown in scenario.GetWorldState: {}".format(
                benchmark_config.config_idx, e))
            self._append_exception(benchmark_config, e)
            return {**benchmark_config.as_dict(),
                    "step": step,
                    "Terminal": "exception_raised"}

        # if behavior is not None (None specifies that also the default model can be evalauted)
        if behavior:
            world.agents[scenario._eval_agent_ids[0]].behavior_model = behavior
        if maintain_history:
            self._append_to_scenario_history(scenario_history, world, scenario)
        self._reset_evaluators(world, scenario._eval_agent_ids, scenario_set_name)
        step_time = parameter_server["Simulation"]["StepTime", "", 0.2]
        if not isinstance(step_time, float):
            step_time = 0.2
        terminal = False
        terminal_why = None
        while not terminal:
            try:
                evaluation_dict = self._get_evalution_dict(world)
            except Exception as e:
                self.logger.error("For config-idx {}, Exception thrown in evaluation: {}".format(
                    benchmark_config.config_idx, e))
                terminal_why = "exception_raised"
                self._append_exception(benchmark_config, e)
                evaluation_dict = {}
                break
            terminal, terminal_why = self._is_terminal(evaluation_dict)
            if not terminal:
                try:
                    world.PlanAgents(step_time)
                except Exception as e:
                    self.logger.error("For config-idx {}, Exception thrown in world.PlanAgents: {}".format(
                        benchmark_config.config_idx, e))
                    terminal_why = "exception_raised"
                    self._append_exception(benchmark_config, e)
                    break

                if viewer:
                    viewer.drawWorld(world, scenario._eval_agent_ids, scenario_idx=benchmark_config.scenario_idx)
                    viewer.drawEvalResults(evaluation_dict)
                    viewer.show(block=False)
                    time.sleep(step_time)
                    viewer.clear()

                try:
                    world.Execute(step_time)
                except Exception as e:
                    self.logger.error("For config-idx {}, Exception thrown in world.Execute: {}".format(
                        benchmark_config.config_idx, e))
                    terminal_why = "exception_raised"
                    self._append_exception(benchmark_config, e)
                    break

                if maintain_history:
                    self._append_to_scenario_history(scenario_history, world, scenario)
                step += 1

        dct = {**benchmark_config.as_dict(),
               "step": step,
               **evaluation_dict,
               "Terminal": terminal_why}

        return dct, scenario_history

    def _append_to_scenario_history(self, scenario_history, world, scenario):
        scenario = Scenario(agent_list=list(world.agents.values()),
                            map_file_name=scenario.map_file_name,
                            eval_agent_ids=scenario.eval_agent_ids,
                            json_params=scenario.json_params)
        scenario_history.append(scenario.copy())

    def _append_exception(self, benchmark_config, exception):
        self.exceptions_caught.append((benchmark_config.config_idx, exception))

    def _reset_evaluators(self, world, eval_agent_ids, scenario_set_name):
        self.initialized_evaluators =\
            self.evaluators.CreateInitializedEvaluators(eval_agent_ids, scenario_set_name)

    def _evaluation_criteria(self):
        bark_evals = self.evaluators.GetEvaluationCriteria()
        bark_evals.append("step")
        return bark_evals

    def _get_evalution_dict(self, world):
        evaluation_dict = {}
        for evaluator_name, evaluator_bark in self.initialized_evaluators.items():
          evaluation_dict[evaluator_name] = evaluator_bark.Evaluate(world)
        return evaluation_dict

    def _is_terminal(self, evaluation_dict):
        terminal = False
        terminal_why = []
        for evaluator_name, function in self.terminal_when.items():
            if function(evaluation_dict[evaluator_name]):
                terminal = True
                terminal_why.append(evaluator_name)
        return terminal, terminal_why

    def _log_eval_average(self, result_dct_list, configs):
        bresult = BenchmarkResult(result_dct_list, configs)
        df = bresult.get_data_frame()
        for eval_group in bresult.get_evaluation_groups():
          if eval_group not in df.columns:
            df[eval_group] = np.nan
        df.fillna(-1, inplace=True)
        grouped = df.apply(pd.to_numeric, errors='ignore').groupby(bresult.get_evaluation_groups()).mean()
        self.logger.info("\n------------------- Current Evaluation Results ---------------------- \n Num. Results:{}\n {} \n \
---------------------------------------------------------------------".format(len(result_dct_list),
                                                                              grouped.to_string()))