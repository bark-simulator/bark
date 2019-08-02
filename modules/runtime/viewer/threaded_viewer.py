# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT


import numpy as np
from bark.geometry import *
from bark.viewer import *
from bark.models.dynamic import *
from modules.runtime.viewer.viewer import BaseViewer
import threading
import time
from timeit import default_timer as timer


class ThreadedViewer(BaseViewer, threading.Thread):
    def __init__(self, renderer, params=None, **kwargs):
        threading.Thread.__init__(self)
        # Load 3d-Model and parameter
        super(ThreadedViewer, self).__init__(params=params)
        self.renderer = renderer
        self.world_queue_list = [] # objects of the world with 
        self.world_queue_world_times = []
        self.render_time = 0.0
        self.render_time_step = 0.03


    def _closest_world_queue_index(self):
        for idx, world_time in enumerate(reversed(self.world_queue_world_times)):
            if self.render_time >= world_time:
                return len(self.world_queue_world_times)- 1 - idx

    def drawWorld(self, world):
        self.world_queue_list.append(world.copy())
        self.world_queue_world_times.append(world.time)

    def reset(self):
        self.world_queue_list.clear()
        self.world_queue_world_times.clear()
        self.render_time = 0.0

        
    def run(self):
        while(True):
            if len(self.world_queue_list) == 0:
                continue
            current_world = self.world_queue_list[self._closest_world_queue_index()]
            executed_world = current_world.world_execution_at_time(self.render_time)
            start = timer()
            self.renderer.drawWorld(executed_world)
            end = timer()
            self.render_time += self.render_time_step
            self.renderer.show(block=False)
            #time.sleep(self.render_time_step - (end-start)) # todo:measure rendering time



