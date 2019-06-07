# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT




from modules.runtime.scenario.scenario_generation.uniform_vehicle_distribution import UniformVehicleDistribution
from modules.runtime.commons.parameters import ParameterServer
from modules.runtime.viewer.pygame_viewer import PygameViewer
import time
import os

scenario_dump_folder = "examples/scenarios/"
scenario_param_file ="highway_merging.json" # must be within scenario dump folder

param_server = ParameterServer(filename= os.path.join("examples/params/",scenario_param_file))

scenario_generation = UniformVehicleDistribution(num_scenarios=3, random_seed=0, params=param_server)
scenario_generation.params.save(os.path.join(scenario_dump_folder,scenario_param_file)) 


viewer = PygameViewer(params=param_server, use_world_bounds=True)
sim_step_time = param_server["simulation"]["step_time",
                                        "Step-time used in simulation",
                                        0.05]
sim_real_time_factor = param_server["simulation"]["real_time_factor",
                                                "execution in real-time or faster", 1]

for _ in range(0,2): # run 5 scenarios in a row, repeating after 3
    scenario, idx = scenario_generation.get_next_scenario()
    world_state = scenario.get_world_state()
    print("Running scenario {} of {}".format(idx, scenario_generation.num_scenarios))
    for _ in range(0, 10): # run each scenario for 3 steps
        world_state.step(sim_step_time)
        viewer.drawWorld(world_state)
        viewer.show(block=False)
        time.sleep(sim_step_time/sim_real_time_factor)

scenario_generation.dump_scenario_list(os.path.join(scenario_dump_folder,"{}.bark_scenarios".format(os.path.splitext(scenario_param_file)[0])))