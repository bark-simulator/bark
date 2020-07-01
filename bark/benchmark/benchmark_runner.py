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

logging.basicConfig()
logging.getLogger().setLevel(logging.INFO)

from bark.runtime.commons.parameters import ParameterServer
from bark.runtime.scenario.scenario import Scenario
from bark.core.world.evaluation import *


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

    def __eq__(self, other):
        return self.config_idx == other.config_idx

    def __hash__(self):
        return self.config_idx

    def get_info_string_list(self):
        info_strings = ["ConfigIdx: {}".format(self.config_idx),
                        "Behavior: {}".format(self.behavior_name),
                        "ScenarioSet: {}".format(self.scenario_set_name),
                        "ScenarioIdx: {}".format(self.scenario_idx)]
        return info_strings


# result of benchmark run
class BenchmarkResult:
    def __init__(self, result_dict, benchmark_configs, histories=None):
        self.__result_dict = result_dict
        self.__benchmark_configs = benchmark_configs
        self.__data_frame = None
        self.__histories = histories or []

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

    def get_history(self, config_idx):
        return self.__histories[config_idx]

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


class BenchmarkRunner:
    def __init__(self,
                 benchmark_database=None,
                 evaluators=None,
                 terminal_when=None,
                 behaviors=None,
                 num_scenarios=None,
                 benchmark_configs=None,
                 logger_name=None,
                 log_eval_avg_every=None):

        self.benchmark_database = benchmark_database
        self.evaluators = evaluators or {}
        self.terminal_when = terminal_when or []
        self.behaviors = behaviors or {}
        self.benchmark_configs = benchmark_configs or \
                                 self._create_configurations(num_scenarios)
        self.exceptions_caught = []
        self.log_eval_avg_every = log_eval_avg_every
        self.logger = logging.getLogger(logger_name or "BenchmarkRunner")
        self.logger.setLevel(logging.INFO)

    def _create_configurations(self, num_scenarios=None):
        benchmark_configs = []
        # run over all scenario generators from benchmark database
        for scenario_generator, scenario_set_name in self.benchmark_database:
            for scenario, scenario_idx in scenario_generator:
                for behavior_name, behavior_bark in self.behaviors.items():
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

    def run(self, viewer=None, maintain_history=False, stage_dir=None):
        results = []
        histories = {}
        for idx, bmark_conf in enumerate(self.benchmark_configs):
            self.logger.info("Running config idx {}/{}: Scenario {} of set \"{}\" for behavior \"{}\"".format(
                idx, len(self.benchmark_configs) - 1, bmark_conf.scenario_idx,
                bmark_conf.scenario_set_name, bmark_conf.behavior_name))
            result_dict, scenario_history = self._run_benchmark_config(copy.deepcopy(bmark_conf), viewer,
                                                                       maintain_history)
            results.append(result_dict)
            histories[bmark_conf.config_idx] = scenario_history
            if self.log_eval_avg_every and (idx + 1) % self.log_eval_avg_every == 0:
                self._log_eval_average(results)
                if stage_dir:
                    stage_result = BenchmarkResult(results, self.benchmark_configs, histories=histories)
                    try:
                        stage_result.dump(os.path.join(stage_dir, "stage_{}_{}.pickle".format(os.getpid(), idx + 1)))
                    except Exception as e:
                        logging.error('Failed to save stage: {}'.format(e))
                        if maintain_history:
                            logging.warning('Retrying without history')
                            stage_result.drop_histories()
                            stage_result.dump(
                                os.path.join(stage_dir, "stage_{}_{}_no_history.pickle".format(os.getpid(), idx + 1)))

        return BenchmarkResult(results, self.benchmark_configs, histories=histories)

    def run_benchmark_config(self, config_idx, **kwargs):
        for idx, bmark_conf in enumerate(self.benchmark_configs):
            if bmark_conf.config_idx == config_idx:
                self.logger.info("Running config idx {}/{}: Scenario {} of set \"{}\" for behavior \"{}\"".format(
                    idx, len(self.benchmark_configs) - 1, bmark_conf.scenario_idx,
                    bmark_conf.scenario_set_name, bmark_conf.behavior_name))
                return self._run_benchmark_config(copy.deepcopy(bmark_conf), **kwargs)
        self.logger.error("Config idx {} not found in benchmark configs. Skipping...".format(config_idx))
        return

    def _run_benchmark_config(self, benchmark_config, viewer=None, maintain_history=False):
        scenario = benchmark_config.scenario
        behavior = benchmark_config.behavior
        parameter_server = ParameterServer(json=scenario._json_params)
        scenario_history = []
        step = 0
        try:
            world = scenario.GetWorldState()
        except Exception as e:
            self.logger.error("For config-idx {}, Exception thrown in scenario.GetWorldState: {}".format(
                benchmark_config.config_idx, e))
            self._append_exception(benchmark_config, e)
            return {"config_idx": benchmark_config.config_idx,
                    "scen_set": benchmark_config.scenario_set_name,
                    "scen_idx": benchmark_config.scenario_idx,
                    "step": step,
                    "behavior": benchmark_config.behavior_name,
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

        dct = {"config_idx": benchmark_config.config_idx,
               "scen_set": benchmark_config.scenario_set_name,
               "scen_idx": benchmark_config.scenario_idx,
               "step": step,
               "behavior": benchmark_config.behavior_name,
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

    def _log_eval_average(self, result_dct_list):
        bresult = BenchmarkResult(result_dct_list, None)
        df = bresult.get_data_frame()
        grouped = df.apply(pd.to_numeric, errors='ignore').groupby(["scen_set", "behavior"]).mean()[
            self._evaluation_criteria()]
        self.logger.info("\n------------------- Current Evaluation Results ---------------------- \n Num. Results:{}\n {} \n \
---------------------------------------------------------------------".format(len(result_dct_list),
                                                                              grouped.to_string()))
