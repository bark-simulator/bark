# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
import matplotlib
from matplotlib.patches import Polygon
from matplotlib import cm
import matplotlib.pyplot as plt

from bark.core.viewer import *
from bark.core.geometry import *
from bark.core.models.dynamic import StateDefinition

from bark.runtime.viewer.viewer import BaseViewer


class MPViewer(BaseViewer):
    # we do not need an init function as pybind11 implements it
    def __init__(self, params=None, **kwargs):
        super(MPViewer, self).__init__(params=params, **kwargs)
        if 'axis' in kwargs:
          self.axes = kwargs.pop("axis")
        else:
          self.axes = plt.subplots()[1]
          # removes whitespace
          plt.subplots_adjust(bottom=0.0, left=0.0, right=1.0, top=1)

    def drawPoint2d(self, point2d, color, alpha):
        self.axes.plot(
            point2d.x(),
            point2d.y(),
            color=self.getColor(color),
            alpha=alpha,
            marker='x')

    def drawLine2d(self, line2d, color='blue', alpha=1.0, dashed=False, zorder=1, linewidth=1):
        lineStyle_string = (0, (5, 10)) if dashed else 'solid'
        line2d_np = line2d.ToArray()
        self.axes.plot(
            line2d_np[:, 0],
            line2d_np[:, 1], 
            lineStyle=lineStyle_string,
            color=self.getColor(color),
            alpha=alpha,
            zorder=zorder,
            linewidth=linewidth)

    def drawPolygon2d(self, polygon, color, alpha, facecolor=None, linewidth=1, zorder=10):
        points = polygon.ToArray()
        polygon_draw = matplotlib.patches.Polygon(
            points,
            True,
            facecolor=self.getColor(facecolor),
            edgecolor=self.getColor(color),
            alpha=alpha,
            linewidth=linewidth,
            zorder=zorder)
        t_start = self.axes.transData
        polygon_draw.set_transform(t_start)
        self.axes.add_patch(polygon_draw)
        center = polygon.center
        self.axes.plot(center[0], center[1], color=self.getColor(color))

    def drawTrajectory(self, trajectory, color):
        if len(trajectory) > 0:
            self.axes.plot(
                trajectory[:, int(StateDefinition.X_POSITION)],
                trajectory[:, int(StateDefinition.Y_POSITION)],
                color=self.getColor(color))

    def drawText(self, position, text, coordinate="axes", **kwargs):
        verticalalignment = kwargs.pop("verticalalignment", "top")
        horizontalalignment = kwargs.pop("horizontalalignment", "center")
        if coordinate=="axes":
            self.axes.text(position[0], position[1], text, horizontalalignment=horizontalalignment,
             verticalalignment=verticalalignment, transform=self.axes.transAxes, **kwargs)
        else:
            self.axes.text(position[0], position[1], text, horizontalalignment='center',
             verticalalignment='top', **kwargs)

    def getColor(self, color):
        if isinstance(color, Viewer.Color):
            return {
                Viewer.Color.White: 'w',
                Viewer.Color.Red: 'red',
                Viewer.Color.Blue: 'darkblue',
                Viewer.Color.Magenta: 'darkgreen',
                Viewer.Color.Brown: 'sienna'
            }.get(color, 'k')
        else:
            return color

    def get_aspect_ratio(self):
      # ratio is width / height
      [w, h] = self._get_ax_size()
      return (w/h)

    def _get_ax_size(self):
      fig = self.axes.get_figure()
      scale = fig.dpi_scale_trans.inverted()
      bbox = self.axes.get_window_extent().transformed(scale)
      width, height = bbox.width, bbox.height
      width *= fig.dpi
      height *= fig.dpi
      return width, height

    def drawWorld(self, world, eval_agent_ids=None, filename=None, scenario_idx=None, debug_text=True):
        self.clear()
        self.axes.set_axis_off()
        super(MPViewer, self).drawWorld(world, eval_agent_ids, filename, scenario_idx, debug_text)
        self._set_visualization_options()
        self.show()
        if filename:
            self.axes.get_figure().savefig(filename)

    def saveFig(self, filename):
      self.axes.get_figure().savefig(filename)

    def show(self, block=False):
        plt.draw()
        if block:
            plt.show(block=True)
        else:
            plt.pause(0.001)

    def _set_visualization_options(self):
        # x and y limits
        self.axes.set_xlim(self.dynamic_world_x_range[0], self.dynamic_world_x_range[1])
        self.axes.set_ylim(self.dynamic_world_y_range[0], self.dynamic_world_y_range[1])
        self.axes.get_xaxis().set_visible(False)
        self.axes.get_yaxis().set_visible(False) 

    def clear(self):
        self.axes.cla()
      
    
    def getColorFromMap(self, float_color):
        return cm.Accent(float_color)
