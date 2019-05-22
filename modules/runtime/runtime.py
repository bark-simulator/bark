# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

from tqdm import trange
from modules.runtime.commons.commons import return_execution_time
from bark.world.opendrive import *
from bark.world import *
from bark.geometry import *


class Runtime(object):
    def __init__(self, step_time, viewer):
        self.step_time = step_time
        self.viewer = viewer

    def reset(self, scenario):
        self.scenario = scenario
        self.world = self.scenario.get_world_state()

    def step(self):
        self.world.step(step_time)

    def render():
        self.viewer = viewer.drawWorld(self.world)
        self.viewer.show(block=False)

    def run(self, steps):
        for step_count in range(steps):
            self.step()
