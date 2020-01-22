# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT




from modules.runtime.scenario.scenario_generation.uniform_vehicle_distribution import UniformVehicleDistribution
from modules.runtime.commons.parameters import ParameterServer
from modules.runtime.viewer.matplotlib_viewer import MPViewer
from modules.runtime.viewer.pygame_viewer import PygameViewer
import time
import os

scenario_param_file ="highway_merging.json" # must be within examples params folder
param_server = ParameterServer(filename= os.path.join("examples/params/",scenario_param_file))
scenario_generation = UniformVehicleDistribution(num_scenarios=3, random_seed=0, params=param_server)

viewer = MPViewer(
  params=param_server,
  x_range=[5060, 5160],
  y_range=[5070,5150],
  use_world_bounds=True)
sim_step_time = param_server["simulation"]["step_time",
                                           "Step-time used in simulation",
                                           0.2]
sim_real_time_factor = param_server["simulation"]["real_time_factor",
                                                  "execution in real-time or faster",
                                                  1]

# TODO(@hart): does not work with bazel test //... because of read only file-system
# scenario_generation.dump_scenario_list(filename="examples/scenarios/highway_merging_dump.bark_scenarios")

# load scenario list
#scenario_generation.load_scenario_list(filename="examples/scenarios/highway_merging_dump.bark_scenarios")

for _ in range(0, 5): # run 5 scenarios in a row, repeating after 3
  scenario, idx = scenario_generation.get_next_scenario()
  world_state = scenario.get_world_state()
  print("Running scenario {} of {}".format(idx, scenario_generation.num_scenarios))
  for _ in range(0, 10): # run each scenario for 10 steps
    world_state.step(sim_step_time)
    viewer.drawWorld(world_state)
    viewer.show(block=False)
    time.sleep(sim_step_time/sim_real_time_factor)
