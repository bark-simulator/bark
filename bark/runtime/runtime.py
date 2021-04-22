# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import logging

import numpy as np
import pandas as pd

from bark.core.world.opendrive import *
from bark.core.world import *
from bark.core.geometry import *
from bark.core.runtime import PyRuntime
from bark.core.models.dynamic import StateDefinition


class Runtime(PyRuntime):
  def __init__(self,
               step_time,
               viewer,
               scenario_generator=None,
               render=False,
               maintain_world_history=False):
    self._step_time = step_time
    self._viewer = viewer
    self._scenario_generator = scenario_generator
    self._scenario_idx = None
    self._scenario = None
    self._render = render
    self._reset_has_been_called = False
    self._maintain_world_history = maintain_world_history
    self._world_history = []

  def reset(self, scenario=None):
    if scenario:
      self._scenario = scenario
    else:
      self._scenario, self._scenario_idx = \
        self._scenario_generator.get_next_scenario()
    self._world = self._scenario.GetWorldState()
    self._reset_has_been_called = True
    self._viewer.Reset()
    self._world_history = []

  def step(self):
    assert(self._reset_has_been_called == True)

    # save history at start of simulation
    if len(self._world_history) == 0 and self._maintain_world_history:
        self._world_history.append(self._world.Copy())

    self._world.PlanAgents(self._step_time)

    if self._render:
        self.render()
    self._world.Execute(self._step_time)
    
    if self._maintain_world_history:
      self._world_history.append(self._world.Copy())

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

  def ExtractTimeSeries(self):
    # world history needs to be saved during simulation!
    state_list = []
    action_list = []
    for w in self._world_history:
        eval_agent = w.GetAgent(self._scenario._eval_agent_ids[0])
        state_list.append(eval_agent.state)
        action_list.append(eval_agent.behavior_model.GetLastAction())
    traj = np.stack(state_list, axis=0)
    act_arr = np.stack(action_list, axis=0)

    d = {'time': traj[:, int(StateDefinition.TIME_POSITION)],
          'x': traj[:, int(StateDefinition.X_POSITION)],
          'y': traj[:, int(StateDefinition.Y_POSITION)],
          'theta': traj[:, int(StateDefinition.THETA_POSITION)],
          'v': traj[:, int(StateDefinition.VEL_POSITION)]}

    # makes this generic for all sorts of input types
    if not np.isscalar(eval_agent.behavior_model.GetLastAction()):
        d["a"] = act_arr[:, 0]
        d["delta"] = act_arr[:, 1]
        if act_arr.shape[1] > 2:
            logging.warning("Incorrect parsing of actions to timeseries")
    else:
        d["acc"] = act_arr        
    df = pd.DataFrame(data=d)
    return df
