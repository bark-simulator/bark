# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

from bark.runtime.scenario.scenario_generation.interaction_dataset_scenario_generation import \
    InteractionDatasetScenarioGeneration
from bark.runtime.commons.parameters import ParameterServer
from bark.runtime.viewer.matplotlib_viewer import MPViewer
from bark.runtime.viewer.video_renderer import VideoRenderer
from bark.examples.paths import Data
import os
import argparse


# set you json config that contains a map and matching tracks.
param_server = ParameterServer(filename=Data.params_data("interaction_example"))
scenario_generation = InteractionDatasetScenarioGeneration(num_scenarios=1,
                                                           random_seed=0,
                                                           params=param_server)

viewer = MPViewer(params=param_server, use_world_bounds=True)

sim_step_time = param_server["simulation"]["step_time",
                                           "Step-time used in simulation",
                                           0.2]
sim_real_time_factor = param_server["simulation"]["real_time_factor",
                                                  "execution in real-time or faster", 1]
scenario = scenario_generation.create_scenarios(param_server, 1)[0]

world_state = scenario.GetWorldState()

sim_time_steps = param_server["simulation"]["simulation_time_steps", "Number of time steps to simulate", 50]
video_renderer = VideoRenderer(renderer=viewer, world_step_time=sim_step_time)

for _ in range(0, sim_time_steps):
  viewer.clear()
  world_state.Step(sim_step_time)
  video_renderer.drawWorld(world_state)

# video_renderer.export_video(filename="/tmp/interaction_dataset", remove_image_dir=True)
    
