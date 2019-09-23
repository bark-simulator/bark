# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import pandas as pd
import logging
logging.getLogger().setLevel(logging.INFO)

class BenchmarkRunner:
    def __init__(self,
               benchmark_database=None,
               evaluators=None,
               terminal_when=None,
               behaviors=None):

        self.benchmark_database = benchmark_database
        self.evaluators = evaluators or {}
        self.terminal_when = terminal_when or []
        self.behaviors = behaviors or {}

        self.dataframe = pd.DataFrame()

    def run(self, num_scenarios=None):
        # run over each behavior
        for behavior_name, behavior_bark in self.behaviors.items():
            # run over all scenario generators from benchmark database
            for scenario_generator, scenario_set_name in self.benchmark_database:
                step_time = scenario_generator.params["simulation"]["step_time"]
                set_name = scenario_generator.params["simulation"]["step_time"]
                for scenario, idx in scenario_generator:
                    if idx > num_scenarios-1:
                        break
                    self._run_scenario(scenario_set_name,
                                       scenario,
                                       idx,
                                       behavior_name,
                                       behavior_bark,
                                       step_time)

                    
    def _run_scenario(self, scenario_set_name, scenario, idx, behavior_name, behavior_bark, step_time):
        logging.info("Running Set {}, Idx {} for behavior {}".format(
            scenario_set_name, idx, behavior_name
        ))
        world = scenario.get_world_state()
        world.agents[scenario._eval_agent_ids[0]].behavior_model = behavior_bark
        self._reset_evaluators(world, scenario._eval_agent_ids)
        terminal = False
        step = 0
        while not terminal:
            evaluation_dict = self._get_evalution_dict(world)
            terminal = self._is_terminal(evaluation_dict)
            self._add_step_result(
                scenario_set_name=scenario_set_name,
                scenario_idx=idx,
                step=step,
                behavior_name=behavior_name,
                evaluator_results=evaluation_dict,
                terminal=terminal)
            world.step(step_time) 
            step += 1

    def _reset_evaluators(self, world, eval_agent_ids):
        for evaluator_name, evaluator_type in self.evaluators.items():
            evaluator_bark = None
            try:
                evaluator_bark = evaluator_type(eval_agent_ids[0])
            except:
                evaluator_bark = evaluator_type()
            world.add_evaluator(evaluator_name, evaluator_bark)

    def _get_evalution_dict(self, world):
        return world.evaluate()

    def _is_terminal(self, evaluation_dict):
        terminal = False
        for evaluator_name, function in self.terminal_when.items():
            terminal = terminal or function(evaluation_dict[evaluator_name])
        return terminal

    def _add_step_result(self,
                     scenario_set_name,
                     scenario_idx,
                     step,
                     behavior_name,
                     evaluator_results,
                     terminal,
                     **kwargs):

        dict = {"Set": scenario_set_name,
                "Idx" : scenario_idx,
                "Step": step,
                "Behavior" : behavior_name,
                **evaluator_results,
                "Terminal": terminal,
                **kwargs}

        self.dataframe = self.dataframe.append(dict, ignore_index=True)