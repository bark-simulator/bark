import numpy as np
from direct.showbase.ShowBase import ShowBase
from direct.gui.DirectGui import DirectButton
from direct.task import Task
from math import pi
import os

from bark.core.viewer import Viewer
from bark.core.geometry import Line2d, Point2d
from bark.core.models.dynamic import StateDefinition
from bark.runtime.viewer.viewer import BaseViewer
from panda3d.core import *

class Panda3dViewer(BaseViewer, ShowBase):
  def __init__(self, params=None, **kwargs):
    super(Panda3dViewer, self).__init__(params=params, **kwargs)
    self._model_path = kwargs.pop(
      "model_path",
      os.path.join(os.path.dirname(os.path.abspath(__file__))) + "/models/car_model.obj")
    self._model_scale = kwargs.pop("model_scale",
                                   np.array([.3,.3,.3], dtype=float))
    self._model_orientation = \
      kwargs.pop("model_orientation",
                 np.array([0, 90, 90],
                 dtype=float))
    self._model_translation = kwargs.pop("model_translation",
                                         np.array([0., 0., 0.], dtype=float))
    self._screen_dims = kwargs.pop("screen_dims", [2*1024, 1024])
    self._line_thickness = kwargs.pop("line_thickness", 0.05)
    self._follow_agent = kwargs.pop("follow_agent", True)
    self._follow_agent_id = kwargs.pop("follow_agent_id", None)
    self._camera_pose = kwargs.pop("camera_pose", [0, 20, 60])
    self._light_pose = kwargs.pop("light_pose", [0, 0, 10000])
    self._camera_orientation = kwargs.pop("camera_pose", [0, 270, 0])
    self._frame_count = 0

    # Panda3d
    self._window = WindowProperties().getDefault()
    self._window.setSize(self._screen_dims[0], self._screen_dims[1])
    self._window.setTitle("BARK Viewer")
    WindowProperties.setDefault(self._window)
    ShowBase.__init__(self)

    self.SetupBasics()
    self._agent_nodes = {}
  
  def SetupBasics(self):
    self.AddCamera()
    self.AddPlane()
    self.AddLight()
    self.AddGenerator()

  def AddCamera(self):
    base.disableMouse()
    # TODO: calculate z based on viewing range

    # self.camera is available due to ShowBase class
    self.camera.setPos(
      self._camera_pose[0],
      self._camera_pose[1],
      self._camera_pose[2])
    self.camera.setHpr(
      self._camera_orientation[2],
      self._camera_orientation[1],
      self._camera_orientation[0])
    
  def AddPlane(self, color=VBase4(1, 1, 1, 1)):
    card = CardMaker("plane")
    card.set_color(color)
    card.setFrame(-10000, 10000, -10000, 10000)
    n = NodePath()
    self.plane = n.attach_new_node(card.generate())
    self.plane.setHpr(0, 270, 0)
    self.plane.reparentTo(self.render)

  def AddLight(self):
    alight = AmbientLight('alight')
    alight.setColor(VBase4(.1, .1, .1, 1))
    self._ambient_light = self.render.attachNewNode(alight)
    self.render.setLight(self._ambient_light)

    # TODO(@hart): position of pt-light needs to be fixed
    plight = PointLight('plight')
    plight.setColor((1., 1., 1., 1))
    self._point_light = self.render.attachNewNode(plight)
    self._point_light.setPos(self._light_pose[0], self._light_pose[1], self._light_pose[2])
    self.render.setLight(self._point_light)

  def AddGenerator(self, budget=1000):
    self._generator = MeshDrawer()
    self._generator.setBudget(budget)
    self._generator_node = self._generator.getRoot()
    self._generator_node.reparentTo(self.render)

  def drawPolygon2d(self, polygon, color, alpha, facecolor=None):
    points = polygon.ToArray()
    self.drawLine2d(polygon, color, alpha)
  
  def drawLine2d(self, line2d, color='blue', alpha=1.0, dashed=False, zorder=1, linewidth=1):
    # TODO: calculate the line thickness based on the z-value
    line2d_np = line2d.ToArray()
    for point in line2d_np:
      self._generator.link_segment(
        Vec3(point[0], point[1], 0.1),
        Vec4(0, 0, 1, 1),
        self._line_thickness,
        self.getColor(Viewer.Color.Blue))
    self._generator.link_segment_end(Vec4(0, 0, 1, 1), self.getColor(Viewer.Color.Blue))

  def get_aspect_ratio(self):
    return self._screen_dims[0] / self._screen_dims[1]
  
  def drawAgent(self, agent, color, alpha, facecolor):
    x_pos = agent.state[int(StateDefinition.X_POSITION)]
    y_pos = agent.state[int(StateDefinition.Y_POSITION)]
    if not agent.id in self._agent_nodes:
      self._agent_nodes[agent.id] = self.loader.loadModel(
        self._model_path)
      self._agent_nodes[agent.id].reparentTo(self.render)
      self._agent_nodes[agent.id].setScale(
        self._model_scale[0], self._model_scale[1], self._model_scale[2])
      self._agent_nodes[agent.id].setColor(self.getColor(Viewer.Color.Blue))
      self._agent_nodes[agent.id].setColorScale(self.getColor(Viewer.Color.Blue))

    angle = ((agent.state[int(StateDefinition.THETA_POSITION)] * 180) / pi +
              self._model_orientation[0])
    self._agent_nodes[agent.id].setHpr(
      angle,
      self._model_orientation[1],
      self._model_orientation[2])
    self._agent_nodes[agent.id].setPos(
      x_pos + self._model_translation[0],
      y_pos + self._model_translation[1],
      self._model_translation[2])

  def getColor(self, color):
    if isinstance(color, Viewer.Color):
      return {
        Viewer.Color.White:
        Vec4(1, 1, 1, 1),
        Viewer.Color.Red:
        Vec4(1, 0, 0, 1),
        Viewer.Color.Blue:
        Vec4(0, 0, 1, 1),
        Viewer.Color.Magenta:
        Vec4(100.0 / 255.0, 1, 100.0 / 255.0, 1),
        Viewer.Color.Brown:
        Vec4(150.0 / 255.0, 50.0 / 255.0, 50.0 / 255.0, 1),
        Viewer.Color.Black:
        Vec4(0, 0, 0, 1)
      }.get(color, Vec4(0, 0, 0, 1))
    else:
      return Vec4(color[0], color[1], color[2], 1)

  def drawWorld(self, world, eval_agent_ids=None, filename=None, scenario_idx=None, debug_text=True):
    self._generator.begin(base.cam, self.render)
    super(Panda3dViewer, self).drawWorld(world, eval_agent_ids, filename, scenario_idx, debug_text)
    
    if self._follow_agent_id is not None:
      self.camera.lookAt(self._agent_nodes[self._follow_agent_id])
    elif isinstance(self._follow_agent, bool) and self._follow_agent == True:
      self.camera.lookAt(self._agent_nodes[eval_agent_ids[0]])

    # self.camera.setHpr(
    #   self._camera_orientation[2],
    #   self._camera_orientation[1],
    #   self._camera_orientation[0])
    base.win.saveScreenshot("/Users/hart/2019/bark/screenshots/screen_{:02d}.png".format((self._frame_count)))
    self._frame_count += 1
    self._generator.end()
    self.taskMgr.step()

  def Reset(self):
    for agent_node in self._agent_nodes.values():
      agent_node.removeNode()
    self._agent_nodes = {}