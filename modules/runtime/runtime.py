# Copyright (c) 2019 fortiss GmbH
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

from tqdm import trange
import copy
from modules.runtime.commons.commons import return_execution_time
from bark.world.opendrive import *
from bark.world import *
from bark.geometry import *


class Runtime(object):
    def __init__(self, world, step_time):
        self.world = world
        self.step_time = step_time

    @return_execution_time
    def step(self, step_time):
        self.world.step(step_time)

    def run(self, steps):
        for step_count in trange(steps, leave=True):
            print("%s: World step took %s seconds." %
                  (step_count, round(self.step(self.step_time), 2)))
