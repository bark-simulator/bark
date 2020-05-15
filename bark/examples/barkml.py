
# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import os
import numpy as np
from bark.runtime.commons.parameters import ParameterServer

from load.benchmark_database import BenchmarkDatabase
from serialization.database_serializer import DatabaseSerializer
from configurations.bark_agent import BARKMLBehaviorModel
from configurations.highway.configuration_lib import HighwayConfiguration

from bark.runtime.viewer.matplotlib_viewer import MPViewer
from load.benchmark_database import BenchmarkDatabase
from bark.benchmark.benchmark_runner import BenchmarkRunner
from bark.core.world.evaluation import EvaluatorGoalReached, \
  EvaluatorCollisionEgoAgent, EvaluatorStepCount
from bark.runtime.commons.parameters import ParameterServer
from bark.runtime.runtime import Runtime

# NOTE(@all): this won't work as a test since we need to have writing access
#             to the summaries folder in bark-ml
# NOTE(@all): need to be in the BARK-ML virtual environment


# some path magic
base_dir = "/home/hart/Dokumente/2020/bark-ml"
params = ParameterServer(filename=base_dir + "/configurations/highway/config.json")
scenario_generation = params["Scenario"]["Generation"]["ConfigurableScenarioGeneration"]
map_filename = scenario_generation["MapFilename"]
scenario_generation["MapFilename"] = base_dir + "/" + map_filename
params["BaseDir"] = base_dir

# actual model
ml_config = HighwayConfiguration(params)
ml_behavior = BARKMLBehaviorModel(configuration=ml_config)

scenario_generator = ml_config._scenario_generator
viewer = MPViewer(params=params, use_world_bounds=True)

env = Runtime(0.2,
              viewer,
              scenario_generator,
              render=True)

env.reset()
env._world.agents[env._scenario._eval_agent_ids[0]].behavior_model = ml_behavior

print(ml_behavior)
for _ in range(0, 50):
  env.step()

# to find database files
os.chdir("../benchmark_database/")
dbs = DatabaseSerializer(test_scenarios=4, test_world_steps=5, num_serialize_scenarios=10)
dbs.process("database")
local_release_filename = dbs.release(version="test")
db = BenchmarkDatabase(database_root=local_release_filename)

evaluators = {"success" : "EvaluatorGoalReached",
              "collision" : "EvaluatorCollisionEgoAgent",
              "max_steps": "EvaluatorStepCount"}
terminal_when = {"collision" :lambda x: x,
                 "max_steps": lambda x : x>31}
behaviors_tested = {"bark_ml": ml_behavior }
                                

benchmark_runner = BenchmarkRunner(benchmark_database=db,
                                   evaluators=evaluators,
                                   terminal_when=terminal_when,
                                   behaviors=behaviors_tested,
                                   log_eval_avg_every=1)

result = benchmark_runner.run()
print(result)




