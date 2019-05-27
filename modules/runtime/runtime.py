# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

from bark.world.opendrive import *
from bark.world import *
from bark.geometry import *


class Runtime(object):
    def __init__(self, step_time, viewer, scenario_generator=None):
        self.step_time = step_time
        self.viewer = viewer
        self.scenario_generator = scenario_generator
        self.scenario_idx = None

    def reset(self, scenario=None):
        if scenario:
            self.scenario = scenario
        else:
            self.scenario, self.scenario_idx = self.scenario_generator.get_next_scenario()
        self.world = self.scenario.get_world_state()

    def step(self):
        self.world.step(step_time)

    def render():
        self.viewer = viewer.drawWorld(self.world)
        self.viewer.show(block=False)

    def run(self, steps):
        for step_count in range(steps):
            self.step()
