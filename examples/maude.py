# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

from maude_interface import make_initial_world, get_ego_agent, apply_action_to_ego_agent
from bark.runtime.viewer.matplotlib_viewer import MPViewer
from bark.runtime.commons.parameters import ParameterServer
from bark.runtime.viewer.video_renderer import VideoRenderer
import time

# list of tuples with acceleration / steering angle
primitives = [(0, 0), (-5, 0), (5, 0)]
world = make_initial_world(primitives)

ego_agent = get_ego_agent(world)
apply_action_to_ego_agent(world, 0) # applies action with idx 0 (keep velocity)

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
    viewer.drawGoalDefinition(ego_agent.goal_definition, "red", 0.5, "red")

    print(ego_agent.state)

    # applies action with idx 1 (break)
    apply_action_to_ego_agent(world, 1)

    time.sleep(sim_step_time/sim_real_time_factor)
