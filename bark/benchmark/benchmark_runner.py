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

logging.basicConfig()
logging.getLogger().setLevel(logging.INFO)

from bark.runtime.commons.parameters import ParameterServer
from bark.runtime.scenario.scenario import Scenario
from bark.benchmark.benchmark_result import BenchmarkResult, BenchmarkConfig, BehaviorConfig
from bark.core.world.evaluation import *
from bark.core.world.evaluation.ltl import *

class BenchmarkRunner:
    def __init__(self,
                 benchmark_database=None,
                 evaluators=None,
                 terminal_when=None,
                 behaviors=None,
                 behavior_configs=None,
                 num_scenarios=None,
                 benchmark_configs=None,
                 logger_name=None,
                 log_eval_avg_every=None,
                 checkpoint_dir=None,
                 merge_existing=False):

        self.benchmark_database = benchmark_database
        self.evaluators = evaluators or {}
        self.terminal_when = terminal_when or []
        if behaviors:
          self.behavior_configs = BehaviorConfig.configs_from_dict(behaviors)
        else:
          self.behavior_configs = behavior_configs or {}
        self.benchmark_configs = benchmark_configs or \
                                 self._create_configurations(num_scenarios)

        self.logger = logging.getLogger(logger_name or "BenchmarkRunner")
        self.logger.setLevel(logging.DEBUG)
        self.logger.info("Total number of {} configs to run".format(len(self.benchmark_configs)))
        self.existing_benchmark_result = BenchmarkResult()
        self.configs_to_run = self.benchmark_configs

        self.checkpoint_dir = checkpoint_dir or "checkpoints"
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
    def merge_checkpoint_benchmark_results(checkpoint_dir):
        checkpoint_files = glob.glob(os.path.join(checkpoint_dir, "**/*.ckpnt"), recursive=True)
        merged_result = BenchmarkResult()
        # merge all checkpoints with new results
        for checkpoint_file in checkpoint_files:
          logging.info("Loading checkpoint {}".format(os.path.abspath(checkpoint_file)))
          next_result = BenchmarkResult.load(os.path.abspath(checkpoint_file), \
              load_configs=True, load_histories=True)
          merged_result.extend(next_result)
        # dump merged result
        if len(merged_result.get_result_dict()) > 0:
          logging.info("Dumping merged result")
          merged_result_filename = os.path.join(checkpoint_dir,"merged_results.ckpnt")
          merged_result.dump(merged_result_filename, \
              dump_configs=True, dump_histories=True)

        # delete checkpoints
        for checkpoint_file in checkpoint_files:
          if checkpoint_file == merged_result_filename:
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

    def _create_configurations(self, num_scenarios=None):
        benchmark_configs = []
        for behavior_config in self.behavior_configs:
            # run over all scenario generators from benchmark database
            for scenario_generator, scenario_set_name, scenario_set_param_desc in self.benchmark_database:
                for scenario, scenario_idx in scenario_generator:
                    if num_scenarios and scenario_idx >= num_scenarios:
                        break
                    benchmark_config = \
                        BenchmarkConfig(
                            len(benchmark_configs),
                            behavior_config,
                            scenario,
                            scenario_idx,
                            scenario_set_name,
                            scenario_set_param_desc
                        )
                    benchmark_configs.append(benchmark_config)
        return benchmark_configs

    def run(self, viewer=None, maintain_history=False, checkpoint_every=None):
        results = []
        histories = {}
        for idx, bmark_conf in enumerate(self.configs_to_run ):
            self.logger.info("Running config idx {} being {}/{}: Scenario {} of set \"{}\" for behavior \"{}\"".format(
                bmark_conf.config_idx, idx, len(self.benchmark_configs) - 1, bmark_conf.scenario_idx,
                bmark_conf.scenario_set_name, bmark_conf.behavior_config.behavior_name))
            result_dict, scenario_history = self._run_benchmark_config(copy.deepcopy(bmark_conf), viewer,
                                                                       maintain_history)
            results.append(result_dict)
            histories[bmark_conf.config_idx] = scenario_history
            if self.log_eval_avg_every and (idx + 1) % self.log_eval_avg_every == 0:
                self._log_eval_average(results, self.configs_to_run)

            if checkpoint_every and (idx+1) % checkpoint_every == 0:
                intermediate_result = BenchmarkResult(results, \
                         self.configs_to_run[0:idx+1], histories=histories)
                checkpoint_file = os.path.join(self.checkpoint_dir, self.get_checkpoint_file_name())
                intermediate_result.dump(checkpoint_file, dump_configs=True, dump_histories=maintain_history)
                self.logger.info("Saved checkpoint {}".format(checkpoint_file))
        benchmark_result = BenchmarkResult(results, self.configs_to_run, histories=histories)
        self.existing_benchmark_result.extend(benchmark_result)
        return self.existing_benchmark_result

    def run_benchmark_config(self, config_idx, **kwargs):
        for idx, bmark_conf in enumerate(self.benchmark_configs):
            if bmark_conf.config_idx == config_idx:
                result_dict, scenario_history = self._run_benchmark_config(copy.deepcopy(bmark_conf), **kwargs)
                return BenchmarkResult(result_dict, [bmark_conf], histories={config_idx : scenario_history})
        self.logger.error("Config idx {} not found in benchmark configs. Skipping...".format(config_idx))
        return

    def _run_benchmark_config(self, benchmark_config, viewer=None, maintain_history=False):
        scenario = benchmark_config.scenario
        behavior = benchmark_config.behavior_config.behavior
        parameter_server = ParameterServer(json=scenario._json_params)
        scenario_history = []
        step = 0
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
        self._reset_evaluators(world, scenario._eval_agent_ids)
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
                if viewer:
                    viewer.drawWorld(world, scenario._eval_agent_ids, scenario_idx=benchmark_config.scenario_idx)
                    viewer.show(block=False)
                    time.sleep(step_time)
                    viewer.clear()
                try:
                    world.Step(step_time)
                except Exception as e:
                    self.logger.error("For config-idx {}, Exception thrown in world.Step: {}".format(
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

    def _reset_evaluators(self, world, eval_agent_ids):
        for evaluator_name, evaluator_params in self.evaluators.items():
            evaluator_bark = None
            if isinstance(evaluator_params, str):
                try:
                    evaluator_bark = eval("{}(eval_agent_ids[0])".format(evaluator_params))
                except:
                    evaluator_bark = eval("{}()".format(evaluator_params))
            elif isinstance(evaluator_params, dict):
                evaluator_bark = eval(
                    "{}(agent_id=eval_agent_ids[0], **evaluator_params['params'])".format(evaluator_params["type"]))
            else:
                raise ValueError
            world.AddEvaluator(evaluator_name, evaluator_bark)

    def _evaluation_criteria(self):
        bark_evals = [eval_crit for eval_crit, _ in self.evaluators.items()]
        bark_evals.append("step")
        return bark_evals

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

    def _log_eval_average(self, result_dct_list, configs):
        bresult = BenchmarkResult(result_dct_list, configs)
        df = bresult.get_data_frame()
        grouped = df.apply(pd.to_numeric, errors='ignore').groupby(bresult.get_evaluation_groups()).mean()[
            self._evaluation_criteria()]
        self.logger.info("\n------------------- Current Evaluation Results ---------------------- \n Num. Results:{}\n {} \n \
---------------------------------------------------------------------".format(len(result_dct_list),
                                                                              grouped.to_string()))
