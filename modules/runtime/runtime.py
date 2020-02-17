# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

from bark.world.opendrive import *
from bark.world import *
from bark.geometry import *
from bark.runtime import PyRuntime


class Runtime(PyRuntime):
  def __init__(self,
               step_time,
               viewer,
               scenario_generator=None,
               render=False):
    self._step_time = step_time
    self._viewer = viewer
    self._scenario_generator = scenario_generator
    self._scenario_idx = None
    self._scenario = None
    self._render = render
    self._reset_has_been_called = False

  def reset(self, scenario=None):
    if scenario:
      self._scenario = scenario
    else:
      self._scenario, self._scenario_idx = \
        self._scenario_generator.get_next_scenario()
    self._world = self._scenario.get_world_state()
    self._reset_has_been_called = True

  def step(self):
    assert(self._reset_has_been_called==True)
    self._world.Step(self._step_time)
    if self._render:
      self.render()

  def render(self):
    # self._viewer.clear()
    self._viewer.drawWorld(
      self._world,
      self._scenario._eval_agent_ids,
      scenario_idx=self._scenario_idx)
    self._viewer.clear()

  def run(self, steps):
    for step_count in range(steps):
      self.Step()
