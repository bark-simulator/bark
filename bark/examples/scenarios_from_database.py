# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

from benchmark_database.load.benchmark_database import BenchmarkDatabase
from bark.runtime.runtime import Runtime
from bark.runtime.viewer.matplotlib_viewer import MPViewer
from bark.runtime.commons.parameters import ParameterServer
import time

                                                
db = BenchmarkDatabase(database_root="external/benchmark_database_release")
try:
  scenario_generation = db.get_scenario_generator(scenario_set_id=0)
except Exception as e:
  print("Error during loading of databse: {}".format(e))
  print("Please perform a release of the benchmark database with the current BARK pickling version.")
  exit()
param_server = ParameterServer()
viewer = MPViewer(
  params=param_server,
  x_range=[5060, 5160],
  y_range=[5070,5150])

env = Runtime(0.2,
              viewer,
              scenario_generation,
              render=True)
env.reset()
for _ in range(0, 5): # run 5 scenarios in a row, repeating after 3
  env.step()