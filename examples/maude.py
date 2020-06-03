# Copyright (c) 2020 Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

from maud_interface import make_initial_world, get_controlled_agent
from modules.runtime.viewer.matplotlib_viewer import MPViewer
from modules.runtime.commons.parameters import ParameterServer
from modules.runtime.viewer.video_renderer import VideoRenderer
import time

world = make_initial_world()

params = ParameterServer()
viewer = MPViewer(params=params, use_world_bounds=True)

# World Simulation
sim_step_time = params["simulation"]["step_time",
                                     "Step-time in simulation", 0.2]
sim_real_time_factor = params["simulation"]["real_time_factor",
                                            "execution in real-time or faster", 1]

# Draw map
video_renderer = VideoRenderer(renderer=viewer, world_step_time=sim_step_time)

for _ in range(0, 20):
    world.Step(sim_step_time)
    viewer.clear()
    viewer.drawWorld(world)
    ego_agent = get_controlled_agent(world)
    print(ego_agent.state)

    viewer.drawGoalDefinition(ego_agent.goal_definition, "red", 0.5, "red")

    time.sleep(sim_step_time/sim_real_time_factor)
