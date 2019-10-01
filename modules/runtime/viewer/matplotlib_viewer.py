# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

from matplotlib.patches import Polygon
import matplotlib.pyplot as plt

from bark.viewer import *
from bark.geometry import *
from modules.runtime.viewer.viewer import BaseViewer


class MPViewer(BaseViewer):
    # we do not need an init function as pybind11 implements it
    def __init__(self, params=None, **kwargs):
        super(MPViewer, self).__init__(params=params, **kwargs)
        self.axes = kwargs.pop("axes", plt.subplots(figsize=(20,20))[1])

    def drawPoint2d(self, point2d, color, alpha):
        self.axes.plot(
            point2d.x(),
            point2d.y(),
            color=self.getColor(color),
            alpha=alpha,
            marker='x')

    def drawLine2d(self, line2d, color='blue', alpha=1.0, dashed=False):
        lineStyle_string = '--' if dashed else '-'
        line2d_np = line2d.toArray()
        self.axes.plot(
            line2d_np[:, 0],
            line2d_np[:, 1], 
            lineStyle=lineStyle_string,
            color=self.getColor(color),
            alpha=alpha)

    def drawPolygon2d(self, polygon, color, alpha):
        points = polygon.toArray()
        polygon_draw = Polygon(
            points,
            True,
            facecolor=self.getColor(color),
            edgecolor=self.getColor(color),
            alpha=alpha)
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
        if coordinate=="axes":
            self.axes.text(position[0], position[1], text, horizontalalignment='center',
             verticalalignment='top', transform=self.axes.transAxes, **kwargs)
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

    def drawWorld(self, world, eval_agent_ids=None, filename=None, scenario_idx=None):
        self.clear()
        super(MPViewer, self).drawWorld(world, eval_agent_ids, filename, scenario_idx)
        self._set_visualization_options()
        self.show()
        if filename:
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
