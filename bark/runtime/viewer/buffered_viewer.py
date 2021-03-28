# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import numpy as np
import logging
from bark.core.viewer import Viewer
from bark.core.geometry import *
from bark.core.models.dynamic import *
from bark.core.world.opendrive import *
from bark.core.world.goal_definition import *
from bark.core.world.renderer import *
from bark.runtime.commons.parameters import ParameterServer
import math

logger = logging.getLogger()


class BufferedViewer:
  def __init__(self, params=None, **kwargs):
    self._params = params or ParameterServer()
    self._renderer = Renderer()

  def DrawAgents(self, world, eval_agent_id):
    ego_agent = world.agents[eval_agent_id]
    for agent in world.agents.values():
      agent_poly = agent.GetPolygonFromState(agent.state)
      agent_primitive = RenderPrimitive(agent_poly)
      # TODO: add styles
      if agent == ego_agent:
        pass
      else:
        pass
      self._renderer.Add(agent_primitive)
      
  def drawWorld(self, world, eval_agent_ids=None):
    self.DrawMap(world)
    self.DrawAgents(world, eval_agent_ids[0])

  def DrawMap(self, world):
    for road in map.GetRoads().values():
      for lane_section in road.lane_sections:
        for lane in lane_section.GetLanes().values():
          self.DrawLane(lane)
      
  def DrawLane(self, lane):
    line_style = "solid"
    if lane.road_mark.type == XodrRoadMarkType.broken or \
      lane.road_mark.type == XodrRoadMarkType.none:
      line_style = dashed
    # TODO: add styles
    lane_primitive = RenderPrimitive(agent_poly)
    self._renderer.Add(lane_primitive)

  def clear(self):
    self._renderer.Clear()
  
  def Reset(self):
    self._renderer.Clear()
