# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
import matplotlib
from matplotlib.patches import Polygon
from matplotlib import cm, colors
import matplotlib.pyplot as plt

import math

import numpy as np

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
          # plt.subplots_adjust(bottom=0.0, left=0.0, right=1.0, top=1)
        self._cmap = self.setupColormap()

    def drawPoint2d(self, point2d, color, alpha):
        self.axes.plot(
            point2d.x(),
            point2d.y(),
            color=self.getColor(color),
            alpha=alpha,
            marker='x')

    def drawLine2d(self, line2d, color='blue', alpha=1.0, dashed=False, zorder=1, **kwargs):
        line_width = kwargs.pop("linewidth", 1)
        marker = kwargs.pop("marker", None)
        marker_size = kwargs.pop("markersize", 12)
        linestyle = (0, (5, 10)) if dashed else 'solid'
        line2d_np = line2d.ToArray()
        self.axes.plot(
            line2d_np[:, 0],
            line2d_np[:, 1],
            linestyle=linestyle,
            color=self.getColor(color),
            alpha=alpha,
            zorder=zorder,
            linewidth=line_width,
            marker=marker,
            markersize = marker_size)

    def drawPolygon2d(self, polygon, color, alpha, facecolor=None, linewidth=1, zorder=10, hatch=''):
        points = polygon.ToArray()
        polygon_draw = matplotlib.patches.Polygon(
            points,
            True,
            facecolor=self.getColor(facecolor),
            edgecolor=self.getColor(color),
            alpha=alpha,
            linewidth=linewidth,
            zorder=zorder,
            hatch=hatch)
        t_start = self.axes.transData
        polygon_draw.set_transform(t_start)
        self.axes.add_patch(polygon_draw)
        center = polygon.center
        self.axes.plot(center[0], center[1], color=self.getColor(color))

    def drawTrajectory(self, trajectory, color='black', **kwargs):
        line_width = kwargs.pop("linewidth", 2)
        line_style = kwargs.pop("linestyle", "-")
        marker = kwargs.pop("marker", None)
        marker_size = kwargs.pop("markersize", 12)
        if len(trajectory) > 0:
            self.axes.plot(
                trajectory[:, int(StateDefinition.X_POSITION)],
                trajectory[:, int(StateDefinition.Y_POSITION)],
                color=self.getColor(color),
                linewidth=line_width,
                linestyle=line_style,
                marker=marker,
                markersize = marker_size)

    def drawArrow(self, pose):
        plt.annotate(s='', xy=(pose[0]+3*math.cos(pose[2]),pose[1]+3*math.sin(pose[2])), xytext=(pose[0],pose[1]), arrowprops=dict(arrowstyle='->'))

    def drawText(self, position, text, coordinate="axes", zorder=11, **kwargs):
        verticalalignment = kwargs.pop("verticalalignment", "top")
        horizontalalignment = kwargs.pop("horizontalalignment", "center")
        zorder = kwargs.pop("zorder", 12)
        t = None
        if coordinate=="axes":
            t = self.axes.text(position[0], position[1], text, horizontalalignment=horizontalalignment,
             verticalalignment=verticalalignment, transform=self.axes.transAxes, zorder=zorder, **kwargs)
        else:
            t = self.axes.text(position[0], position[1], text, horizontalalignment='center',
             verticalalignment='top', zorder=zorder, **kwargs)
        return t


    def drawCircle(self, position, radius):
        circle = plt.Circle(position, radius, color='r')
        self.axes.add_artist(circle)

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

    def drawWorld(self, world, eval_agent_ids=None, filename=None, scenario_idx=None, debug_text=True, axes_visible=False):
        self.clear()
        if not axes_visible:
            self.axes.set_axis_off()
        super(MPViewer, self).drawWorld(world, eval_agent_ids, filename, scenario_idx, debug_text)
        self._set_visualization_options(axes_visible)
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

    def _set_visualization_options(self, axes_visible):
        # x and y limits
        self.axes.set_xlim(self.dynamic_world_x_range[0], self.dynamic_world_x_range[1])
        self.axes.set_ylim(self.dynamic_world_y_range[0], self.dynamic_world_y_range[1])
        self.axes.get_xaxis().set_visible(axes_visible)
        self.axes.get_yaxis().set_visible(axes_visible) 

    def clear(self):
        self.axes.cla()

    def getSizeOfColormap(self):
        return 20 # cm.tab20 has 20 entries

    def setupColormap(self):
        cmap_np = cm.tab20(np.linspace(0, 1, 20)) # tab20 has 20 colours
        cmap_np = np.delete(cmap_np, [6,7], 0) # remove red (reserved for ego)
        return colors.ListedColormap(cmap_np)

    def getColorFromMap(self, double_color):
        return self._cmap(double_color)
