# Copyright (c) 2020 Julian Bernhard, Klemens Esterle, Patrick Hart and
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
from bark.core.world.evaluation import *

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

    def __eq__(self, other):
        return self.config_idx == other.config_idx

    def __hash__(self):
        return self.config_idx

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

    def drop_histories(self):
        self.__histories.clear()

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
        # BenchmarkResult._sort_bench_confs(benchmark_configs)
        b_conf_list = list(filter(lambda x: x.config_idx == config_idx, benchmark_configs))
        bench_conf = None
        if len(b_conf_list) == 0:
            logging.error("Benchmark config {} not found!".format(config_idx))
        else:
            bench_conf = list(b_conf_list)[0]
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
    def _load_and_merge(file_name, filetype):
        pass
    
    def _save_and_split(file_name, filetype, )

    def dump(self, filename, dump_configs=True):
        if isinstance(self.__data_frame, pd.DataFrame):
            self.__data_frame = None
        if not dump_configs:
            self.__benchmark_configs = None
        try:
            with open(filename, 'wb') as handle:
                pickle.dump(self, handle, protocol=pickle.HIGHEST_PROTOCOL)
        except Exception as e:
            logging.error('Failed to write results to {}!\nError: {}\nRetrying without history.'.format(filename, e))
            self.drop_histories()
            (root, ext) = os.path.splitext(filename)
            filename = "{}_no_histories{}".format(root, ext)
            with open(filename, 'wb') as handle:
                pickle.dump(self, handle, protocol=pickle.HIGHEST_PROTOCOL)
        logging.info('Saved BenchmarkResult to {}'.format(
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

    @staticmethod
    def merge_checkpoint_benchmark_results(checkpoint_dir):
        checkpoint_files = glob.glob(os.path.join(checkpoint_dir, "**/*.ckpnt"), recursive=True)
        merged_result = BenchmarkResult()
        # merge all checkpoints with new results
        for checkpoint_file in checkpoint_files:
          logging.info("Loading checkpoint {}".format(os.path.abspath(checkpoint_file)))
          next_result = BenchmarkResult.load(os.path.abspath(checkpoint_file))
          merged_result.extend(next_result)
        # dump merged result
        if len(merged_result.get_result_dict()) > 0:
          logging.info("Dumping merged result")
          merged_result_filename = os.path.join(checkpoint_dir,"merged_results.ckpnt")
          merged_result.dump(merged_result_filename)

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
            for scenario_generator, scenario_set_name in self.benchmark_database:
                for scenario, scenario_idx in scenario_generator:
                    if num_scenarios and scenario_idx >= num_scenarios:
                        break
                    benchmark_config = \
                        BenchmarkConfig(
                            len(benchmark_configs),
                            behavior_config,
                            scenario,
                            scenario_idx,
                            scenario_set_name
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
                intermediate_result.dump(checkpoint_file)
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
        for evaluator_name, evaluator_type in self.evaluators.items():
            evaluator_bark = None
            try:
                evaluator_bark = eval("{}(eval_agent_ids[0])".format(evaluator_type))
            except:
                evaluator_bark = eval("{}()".format(evaluator_type))
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
