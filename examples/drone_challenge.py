# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

# ffmpeg must be installed


from modules.runtime.scenario.scenario_generation.drone_challenge import DroneChallengeScenarioGeneration
from modules.runtime.commons.parameters import ParameterServer
from modules.runtime.viewer.matplotlib_viewer import MPViewer
from modules.runtime.viewer.video_renderer import VideoRenderer
import os

param_server = ParameterServer()
scenario_generation = DroneChallengeScenarioGeneration(
  num_scenarios=3, random_seed=0, params=param_server)


viewer = MPViewer(params=param_server, x_range=[-30, 30], y_range=[-30, 30])
sim_step_time = param_server["simulation"]["step_time",
                                           "Step-time used in simulation",
                                           0.2]
sim_real_time_factor = param_server["simulation"]["real_time_factor",
                                                  "execution in real-time or faster", 1]
scenario, idx = scenario_generation.get_next_scenario()


for _ in range(0, 5): # run 5 scenarios in a row, repeating after 3
  scenario, idx = scenario_generation.get_next_scenario()
  world = scenario.get_world_state()
  # 20 steps
  for _ in range(0, 20):
    world.step(sim_step_time)
    viewer.drawWorld(world)
  
