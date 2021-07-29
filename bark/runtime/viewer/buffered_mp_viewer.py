# Copyright (c) 2021 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from bark.core.models.dynamic import *
from bark.core.world.renderer import *
from bark.runtime.commons.parameters import ParameterServer
from bark.runtime.viewer.buffered_viewer import BufferedViewer


class BufferedMPViewer(BufferedViewer):
  def __init__(self, params=None, ax=None, **kwargs):
    super().__init__(params=params)
    self._ax = ax or plt.subplots()[1]
    self._vparams = self._params["Visualization"]
    self._dx = self._vparams["dx", "view range x", 20]
    self._dy = self._vparams["dy", "view range y", 20]
  def drawWorld(self, world, eval_agent_ids=None, scenario_idx=None):
    self._ax.clear()
    super().drawWorld(world, eval_agent_ids, scenario_idx)
    self.render(world)
    world.renderer.Clear()
  def render(self, world):
    for type_name, primitives in world.renderer.primitives.items():
      if type_name == "EGO_AGENT_STATE":
        self.ProcessEgoState(primitives)
      elif type_name == "MAP_LINE" or type_name == "LINES":
        self.DrawLines(primitives)
      elif type_name == "EGO_AGENT" or type_name == "OTHER_AGENT" \
        or type_name == "POLYGONS":
        self.DrawPolygons(primitives)
      elif type_name == "POINTS":
        self.DrawPoints(primitives)
    plt.tight_layout()
    plt.axis('off')
    plt.pause(0.001)

  def ProcessEgoState(self, primitives):
    ego_state = primitives[-1].object
    self._ax.set_xlim([ego_state[1] - self._dx, ego_state[1] + self._dx])
    self._ax.set_ylim([ego_state[2] - self._dy, ego_state[2] + self._dy])

  def DrawLines(self, primitives):
    for primitive in primitives:
      np_pts = primitive.object.ToArray()
      color = np.array(primitive.conf["stroke_color"])/255
      self._ax.plot(np_pts[:, 0], np_pts[:, 1], color=color)

  def DrawFilledPolygon(self, primitive):
    points = primitive.object.ToArray()
    polygon_draw = matplotlib.patches.Polygon(
      points,
      True,
      facecolor=np.array(primitive.conf["fill_color"])/255,
      edgecolor=np.array(primitive.conf["stroke_color"])/255,
      linewidth=0.2)
    t_start = self._ax.transData
    polygon_draw.set_transform(t_start)
    self._ax.add_patch(polygon_draw)
    color = np.array(primitive.conf["stroke_color"])/255
    self._ax.plot(points[:, 0], points[:, 1], color=color)

  def DrawPolygons(self, primitives):
    for primitive in primitives:
      self.DrawFilledPolygon(primitive)

  def DrawPoints(self, primitives):
    for p in primitives:
      color = np.array(primitive.conf["stroke_color"])/255
      plt.plot(p.object.x(), p.object.y(), marker='*', color=color)