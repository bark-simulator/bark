
# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import os
import numpy as np
from modules.runtime.commons.parameters import ParameterServer

from configurations.bark_agent import BARKMLBehaviorModel
from configurations.sac_highway.configuration import SACHighwayConfiguration
from modules.runtime.viewer.matplotlib_viewer import MPViewer
from load.benchmark_database import BenchmarkDatabase
from modules.benchmark.benchmark_runner import BenchmarkRunner
from bark.world.evaluation import EvaluatorGoalReached, \
  EvaluatorCollisionEgoAgent, EvaluatorStepCount
from modules.runtime.commons.parameters import ParameterServer
from modules.runtime.runtime import Runtime

# NOTE(@all): this won't work as a test since we need to have writing access
#             to the summaries folder in bark-ml
# NOTE(@all): need to be in the BARK-ML virtual environment


# some path magic
base_dir = "/home/hart/Dokumente/2020/bark-ml"
params = ParameterServer(filename=base_dir + "/configurations/highway/config_five.json")
scenario_generation = params["Scenario"]["Generation"]["DeterministicScenarioGeneration"]
map_filename = scenario_generation["MapFilename"]
scenario_generation["MapFilename"] = base_dir + "/" + map_filename
params["BaseDir"] = base_dir

# actual model
sac_configuration = SACHighwayConfiguration(params)
ml_behavior = BARKMLBehaviorModel(configuration=sac_configuration)

scenario_generator = sac_configuration._scenario_generator
viewer = MPViewer(params=params, x_range=[5060, 5160], y_range=[5070, 5150])

env = Runtime(0.2,
              viewer,
              scenario_generator,
              render=True)

env.reset()
env._world.agents[env._scenario._eval_agent_ids[0]].behavior_model = ml_behavior

print(ml_behavior)
for _ in range(0, 30):
  env.step()


# db
# db = BenchmarkDatabase(database_root="./examples/scenarios/benchmark_database_0.0.1.zip")
# evaluators = {"success" : EvaluatorGoalReached,
#               "collision" : EvaluatorCollisionEgoAgent,
#               "max_steps": EvaluatorStepCount}
# terminal_when = {"collision" :lambda x: x,
#                  "max_steps": lambda x : x>2}
# # params = ParameterServer(filename= os.path.join("examples/params/", scenario_param_file))
# behaviors_tested = {"bark_ml": ml_behavior }
                                

# benchmark_runner = BenchmarkRunner(benchmark_database=db,
#                                    evaluators=evaluators,
#                                    terminal_when=terminal_when,
#                                    behaviors=behaviors_tested)

# benchmark_runner.run(1) 

# # benchmark_runner.dataframe.to_pickle("uct_planner_results.pickle")




