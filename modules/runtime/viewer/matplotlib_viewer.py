# Copyright (c) 2019 fortiss GmbH
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import matplotlib
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
import matplotlib.pyplot as plt

from bark.viewer import *
from bark.geometry import *
from modules.runtime.viewer.viewer import BaseViewer


class MPViewer(BaseViewer):
    # we do not need an init function as pybind11 implements it
    def __init__(self, params=None, axes=None):
        super(MPViewer, self).__init__(params=params)
        # setup matplot lib figure
        # scene rectangle -> to be defined
        # other parameters (line width scaling, ...)
        if axes is None:
            fig, ax = plt.subplots()
        else:
            ax = axes
        self.axes = ax
        self.axes.set_aspect('equal')
        self.sw_corner = [0, 0]
        self.ne_corner = [20, 20]

    def drawPoint2d(self, point2d, color, alpha):
        self.axes.plot(
            point2d.x(),
            point2d.y(),
            color=self.getColor(color),
            alpha=alpha,
            marker='x')

    def drawLine2d(self, line2d, color='blue', alpha=1.0):
        line2d_np = line2d.toArray()
        self.axes.plot(
            line2d_np[:, 0],
            line2d_np[:, 1],
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
        self.axes.plot(center[0], center[1], 'o', color=self.getColor(color))

    def drawTrajectory(self, trajectory, color):
        if len(trajectory) > 0:
            self.axes.plot(
                trajectory[:, int(StateDefinition.X_POSITION)],
                trajectory[:, int(StateDefinition.Y_POSITION)],
                color=self.getColor(color))

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

    def show(self, block=True):
        self.axes.set_aspect('equal')
        plt.draw()
        if block:
            plt.show(block=True)
        else:
            plt.pause(0.05)

    def clear(self):
        plt.gca().cla()
