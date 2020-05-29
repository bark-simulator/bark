# Copyright (c) 2020 fortiss GmbH
# 
# Based on the implementation by Luis Gressenbuch
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

from bark.runtime.scenario.scenario_generation.interaction_dataset_scenario_generation import \
    InteractionDatasetScenarioGeneration
from bark.runtime.commons.parameters import ParameterServer
from bark.runtime.viewer.matplotlib_viewer import MPViewer
from bark.runtime.viewer.video_renderer import VideoRenderer
import os
import argparse

# Parse scenario file name
parser = argparse.ArgumentParser(description="Interaction dataset file in bark")
parser.add_argument("scenario_file", metavar="scenario_name", type=str, help="name of the scenario parameter file")
args = parser.parse_args()
scenario_param_file = args.scenario_file  # must be within examples params folder
param_server = ParameterServer(filename=os.path.join(os.path.dirname(__file__),"params/", scenario_param_file))

scenario_generation = InteractionDatasetScenarioGeneration(num_scenarios=1, random_seed=0, params=param_server)

#viewer = MPViewer(params=param_server, x_range=[-234 + 1114, -105 + 1114], y_range=[-130 + 1107, -73 + 1107])
#viewer = MPViewer(params=param_server)
viewer = MPViewer(params=param_server, use_world_bounds=True)

sim_step_time = param_server["simulation"]["step_time",
                                           "Step-time used in simulation",
                                           0.2]
sim_real_time_factor = param_server["simulation"]["real_time_factor",
                                                  "execution in real-time or faster", 1]
scenario = scenario_generation.create_single_scenario()

world_state = scenario.get_world_state()

sim_time_steps = param_server["simulation"]["simulation_time_steps", "Number of time steps to simulate", 40]
for _ in range(0, sim_time_steps):
    world_state.DoPlanning(sim_step_time)
    viewer.drawWorld(world_state, scenario._eval_agent_ids)
    viewer.show(block=False)
    viewer.clear()
    world_state.DoExecution(sim_step_time)

    
