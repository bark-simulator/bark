# Copyright (c) 2020 Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

from modules.runtime.scenario.scenario_generation.interaction_dataset_scenario_generation import \
    InteractionDatasetScenarioGeneration
from modules.runtime.commons.parameters import ParameterServer
from modules.runtime.viewer.matplotlib_viewer import MPViewer
from modules.runtime.viewer.video_renderer import VideoRenderer
import os
import argparse


# set you json config that contains a map and matching tracks.
param_server = ParameterServer(filename=os.path.join("examples/params/interaction_example.json"))
scenario_generation = InteractionDatasetScenarioGeneration(num_scenarios=1,
                                                           random_seed=0,
                                                           params=param_server)

viewer = MPViewer(params=param_server, use_world_bounds=True)

sim_step_time = param_server["simulation"]["step_time",
                                           "Step-time used in simulation",
                                           0.2]
sim_real_time_factor = param_server["simulation"]["real_time_factor",
                                                  "execution in real-time or faster", 1]
scenario = scenario_generation.create_single_scenario()

world_state = scenario.GetWorldState()

sim_time_steps = param_server["simulation"]["simulation_time_steps", "Number of time steps to simulate", 50]
video_renderer = VideoRenderer(renderer=viewer, world_step_time=sim_step_time)

for _ in range(0, sim_time_steps):
  world_state.DoPlanning(sim_step_time)
  viewer.clear()
  video_renderer.drawWorld(world_state)
  world_state.DoExecution(sim_step_time)

# video_renderer.export_video(filename="./interaction_dataset", remove_image_dir=True)
    
