# Copyright (c) 2021 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
import math
import numpy as np
import logging
from bark.core.viewer import Viewer
from bark.core.geometry import *
from bark.core.models.dynamic import *
from bark.core.world.opendrive import *
from bark.core.world.goal_definition import *
from bark.core.world.renderer import *
from bark.runtime.commons.parameters import ParameterServer


class BufferedViewer:
  def __init__(self, params=None, **kwargs):
    self._params = params or ParameterServer()
    self._renderer = Renderer()

  def DrawAgents(self, world, eval_agent_id):
    ego_agent = world.agents[eval_agent_id]
    for agent in world.agents.values():
      agent_poly = agent.GetPolygonFromState(agent.state)
      # TODO: add styles
      if agent == ego_agent:
        agent_primitive = RenderPrimitive(agent_poly, "EGO_AGENT")
        agent_primitive.conf["line_color"] = "blue"
        agent_primitive.conf["face_color"] = "green"
      else:
        agent_primitive = RenderPrimitive(agent_poly, "OTHER_AGENT")
        agent_primitive.conf["line_color"] = "yellow"
        agent_primitive.conf["face_color"] = "red"
      world.renderer.Add(agent_primitive)
      
  def drawWorld(self, world, eval_agent_ids=None, scenario_idx=None):
    world.renderer = self._renderer
    self.DrawMap(world)
    self.DrawAgents(world, eval_agent_ids[0])
    
    # add velocity
    ego_agent = world.agents[eval_agent_ids[0]]
    ego_agent_state = ego_agent.state
    ego_vel_primitive = RenderPrimitive(
      ego_agent_state[int(StateDefinition.VEL_POSITION)],
      "EGO_AGENT_VEL")
    world.renderer.Add(ego_vel_primitive)

  def DrawMap(self, world):
    map = world.map.GetOpenDriveMap()
    for road in map.GetRoads().values():
      for lane_section in road.lane_sections:
        for lane in lane_section.GetLanes().values():
          self.DrawLane(lane, world)
      
  def DrawLane(self, lane, world):
    lane_primitive = RenderPrimitive(lane.line, "MAP")
    # TODO: add styles
    lane_primitive.conf["line_color"] = "blue"
    if lane.road_mark.type == XodrRoadMarkType.broken or \
      lane.road_mark.type == XodrRoadMarkType.none:
      lane_primitive.conf["line_color"] = "gray"
    world.renderer.Add(lane_primitive)

  def clear(self):
    self._renderer.Clear()
  
  def Reset(self):
    self._renderer.Clear()
