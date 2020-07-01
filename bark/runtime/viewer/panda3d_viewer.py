import numpy as np
from direct.showbase.ShowBase import ShowBase
from direct.gui.DirectGui import DirectButton
from direct.task import Task
from math import pi
import os

from bark.core.viewer import Viewer
from bark.core.models.dynamic import StateDefinition
from bark.runtime.viewer.viewer import BaseViewer
from panda3d.core import WindowProperties, MeshDrawer, VBase4, VBase3,Vec4, Vec3, Mat4, AmbientLight, CardMaker, NodePath


class Panda3dViewer(BaseViewer, ShowBase):
    def __init__(self, params=None, **kwargs):
        # Load 3d-Model and parameter
        super(Panda3dViewer, self).__init__(params=params)
        self.world_x_range = kwargs.pop("x_range", [-40, 40])
        self.world_y_range = kwargs.pop("y_range", [-40, 40])
        self.follow_agent_id = kwargs.pop("follow_agent_id", True)
        self.perspective = kwargs.pop("perspective", -3)
        self.screen_dims = kwargs.pop("screen_dims", [1024, 1024])
        self.path = os.path.join(os.path.dirname(os.path.abspath(__file__)))
        self.agent_model_path = kwargs.pop("model_path",
                                           self.path + "/models/car.obj")
        self.texture_path = kwargs.pop("model_path",
                                           self.path + "/models/white_texture.png")
        self.agent_scale = kwargs.pop("model_scale",
                                      np.array([1.,1.,1.], dtype=float))
        self.agent_orientation = \
            kwargs.pop("model_orientation",np.array([0, 90, 90],
                       dtype=float))
        self.agent_translation = kwargs.pop("model_translation",
                                            np.array([0., 0.], dtype=float))
        self.range_for_zoom = kwargs.pop("is_zoom_range", False)
        self.line_thicknesses = kwargs.pop("line_thickness", { # Dict of keys cameras and values [line thickness, height] which are needed to calculate the dynamic thickness
            -3: [0.03, 10],
            -2: [0.03, 10],
            -1: [0.35, 700],
            0: [0.05, 70]
        })
        # Parameter for agent camera views
        # [driving direction offset, height, x-orientation,y-orientation,z-orientation offset to driving direction, LookAt?]
        self.agent_cam_parameter = kwargs.pop(
            "agent_view", {
                "bird_agent": [0, 70, 0, 270, 270, False],
                "third": [-35, 15, 0, 0, 0, True],
                "first": [1.2, 2.7, 180, 190, 90, False]
            })

        # Set Window Size
        self.wp = WindowProperties().getDefault()
        self.wp.setSize(self.screen_dims[0], self.screen_dims[1])
        self.wp.setTitle("BARK Panda3d Viewer")
        WindowProperties.setDefault(self.wp)
        ShowBase.__init__(self)

        # -2 : global camera with camera control, -1: transition state for enabling mouse control, 0: agent cameras
        self.perspectives = {
            -3: ["third", "autozoom"],
            -2: ["bird_agent", "autozoom"],
            -1: ["None"],
            0: ["bird_agent", "third", "first"]
        }
        

        # Render Objects Dict
        self.agent_nodes = {}
        # Agent Poses Dict
        self.agent_poses = {}
        self.setLight()

        # Creating a plane as floor #TODO parameter from map parameter
        frame = [
            -self.screen_dims[0] / 2, self.screen_dims[0] / 2,
            -self.screen_dims[1] / 2, self.screen_dims[1] / 2
        ]
        self.createPlane(frame=frame, color=VBase4(1, 1, 1, 1))

        # Set up the camera loop task
        self.camera_list = [-1]
        self.initCam()
        self.taskMgr.add(self.spinCameraTask, "SpinCameraTask")

        # Set up the line generator
        self.setDrawer(budget=100000)

        self.perspective = self.perspectives[self.camIndex(self.perspective)]
        self.line_thickness = self.line_thicknesses[self.camIndex(self.perspective)][0]
        self.addButtons()

    def setDrawer(self, budget=100000):
        """Initializes the MeshDrawer() generator to draw lines in Panda3d

        Keyword Arguments:
            budget {int} -- maximum triangles for rendering the mesh (default: {100000})
        """
        self.generator = MeshDrawer()
        self.generator.setBudget(budget)
        self.generatorNode = self.generator.getRoot()
        self.generatorNode.reparentTo(self.render)

    def createPlane(self, frame=None, color=VBase4(1, 1, 11, 1)):
        """ Creates a Plane/Card with the Panda3d Cardmaker() class
        Keyword Arguments:
            frame {list} -- The coordinates [x1,y1,x2,y2] of the planes/cards edges (default: {[-1, -1, 1, 1]})
            color {VBase4} -- The color of the planes/cards (default: {VBase4(1, 1, 1, 1)})
        """
        frame = frame or [-1, -1, 1, 1]
        card = CardMaker("plane")
        card.set_color(color)
        card.set_frame(frame[0], frame[1], frame[2], frame[3])
        n = NodePath()
        self.plane = n.attach_new_node(card.generate())
        self.plane.setHpr(0, 270, 0)
        self.plane.reparentTo(self.render)


    def setLight(self, color=VBase4(.1, .1, .1, 1)):
        """Sets an ambient and omnidirectional light for rendering
        Keyword Arguments:
            color {VBase4} -- color of the ambient light (default: {VBase4(1, 1, 1, 1)})
        """
        alight = AmbientLight('alight')
        alight.setColor(color)
        alnp = self.render.attachNewNode(alight)
        self.render.setLight(alnp)

        # TODO(@hart): position of pt-light needs to be fixed
        plight = PointLight('plight')
        plight.setColor((1, 1, 1, 1))
        self.plnp = self.render.attachNewNode(plight)
        self.plnp.setPos(0, 0, 10000)
        self.render.setLight(self.plnp)


    def addButtons(self):
        self.cam_btn = DirectButton(
            text="Camera",
            scale=0.05,
            command=self.switchCamera,
            pos=Vec3(0.9, 0, 0.9))
        self.per_btn = DirectButton(
            text="View",
            scale=0.05,
            command=self.switchView,
            pos=Vec3(0.9, 0, 0.85))

    def camIndex(self, follow_id):
        return follow_id

    def switchCamera(self):
        # Switches between global and agent cameras
        self.follow_agent_id = self.camera_list.pop(0)
        self.camera_list.append(self.follow_agent_id)
        self.perspective = self.perspectives[self.camIndex(
            self.follow_agent_id)].copy()
        self.line_thickness = self.line_thicknesses[self.camIndex(
            self.follow_agent_id)][0]

    def switchView(self):
        # Switches between the perspectives defined by the camera
        self.perspective.append(self.perspective[0])
        self.perspective.pop(0)

    def updateCamera(self, lookAt=False):
        """Updates the camera position from calls using variables self.cam_pose and self.cam_or

        Keyword Arguments:
            lookAt {bool} -- If true the orientation is calculated by the agents position (default: {False})
        """

        self.camera.setPos(self.cam_pose[0], self.cam_pose[1],
                           self.cam_pose[2])
        if lookAt:
            self.camera.lookAt(self.agent_nodes[self.follow_agent_id])
        else:
            self.camera.setHpr(self.cam_or[2], self.cam_or[1], self.cam_or[0])

    def spinCameraTask(self, task):
        """This function sets and updates the camera according to environment variables and parameters

        Arguments:
            task {task} -- Panda3d Task

        Returns:
            Task.cont -- Panda3d Task Return
        """
        if self.follow_agent_id is -1:  # Transition into camera control view
            self.setMouseControl()
        if self.follow_agent_id is -2:  # Global Camera
            if (self.perspective[0] is self.perspectives[-2][1]):
                self.setAutoZoomCam(self.agent_poses)
        if self.follow_agent_id >= 0:  # Camera for all agents
            self.setAgentCam(self.perspective[0],
                             self.agent_poses[self.follow_agent_id])
        return Task.cont
    
    def initCam(self, poses=None, orientation=None):#TODO Calculate from map parameter before
        poses = poses or [0, 0, 700]
        orientation = orientation or [0, 270, 0]
        self.cam_pose = np.array(poses, dtype=float)
        self.cam_or = np.array(orientation, dtype=float)

    def setMouseControl(self):
        # Enables Panda3d Standard Mouse Control
        # Right mouse button: zoom, left mouse Button: move, middle mouse button: tilt
        self.initCam()
        self.updateCamera(lookAt=False)
        self.mat = Mat4(self.camera.getMat())
        self.mat.invertInPlace()
        base.mouseInterfaceNode.setMat(self.mat)
        base.enableMouse()
        # self.follow_agent_id = -2
        self.perspective = self.perspectives[self.camIndex(
            self.follow_agent_id)].copy()

    def setAutoZoomCam(self, agent_poses):
        """This function calculates the camera position so that all agents are visible

        Arguments:
            agent_poses {[dict]} -- [look up table of all agent poses]
        """
        points = np.array([[]], dtype=float)
        if ((self.world_x_range is not None)
                and (self.world_y_range is not None) and self.range_for_zoom):
            points = np.array([[self.world_x_range[0], self.world_y_range[0]],
                               [self.world_x_range[1], self.world_y_range[1]]])
        # Build concat matrix of all positions
        for _, agent in agent_poses.items():
            if points.size is 0:
                points = np.array([[agent[0], agent[1]]], dtype=float)
            else:
                points = np.append(points, [[agent[0], agent[1]]], axis=0)
        # Calculate maximum distance between min,max of x or y poses
        max_dist = np.max(np.max(points, axis=0) - np.min(points, axis=0))
        # Calculate mean positions as center for camera
        center = (np.max(points, axis=0) + np.min(points, axis=0)) / 2
        # Lower bound for vieweable area
        max_dist = np.maximum(max_dist * 1.25, 40)
        # Calculate camera height
        zoom = max_dist * np.tan(pi * (60 / 180))
        self.cam_pose = np.array([center[0], center[1], zoom], dtype=float)
        self.cam_or = np.array([0, 270, 0], dtype=float)
        self.updateCamera(lookAt=False)

    def setAgentCam(self, perspective, agent_poses):
        """Sets up the class variable for the camerea position and orientation

        Arguments:
            perspective {list} -- List of strings describing the perspective currently ["bird_agent","third","first"] which is used as an index for the parameters dict
            agent_poses {np.array([x,y,theta])} -- Describes the agent position
        """

        base.disableMouse()
        self.cam_pose[0] = agent_poses[0] + self.agent_cam_parameter[
            perspective][0] * np.cos(agent_poses[2])
        self.cam_pose[1] = agent_poses[1] + self.agent_cam_parameter[
            perspective][0] * np.sin(agent_poses[2])
        self.cam_pose[2] = self.agent_cam_parameter[perspective][1]
        self.cam_or = np.array([
            self.agent_cam_parameter[perspective][2],
            self.agent_cam_parameter[perspective][3],
            (agent_poses[2] * 180) / pi +
            self.agent_cam_parameter[perspective][4]
        ],
                               dtype=float)
        self.updateCamera(lookAt=self.agent_cam_parameter[perspective][5])

    def calcLineThickness(self,cameras=None):
        """Uses a linear approximation between two cameras to calculate the line thickness

        Arguments:
            cameras {int array} -- Sets the indices of the target cameras
        Returns:
            [float] -- [The approximated thickness]
        """
        cameras = cameras or [-1,0]
        incline = (self.line_thicknesses[cameras[0]][0] - self.line_thicknesses[cameras[1]][0]) / (self.line_thicknesses[cameras[0]][1]-self.line_thicknesses[cameras[1]][1])
        return np.maximum((self.line_thicknesses[cameras[1]][0] + incline * (self.camera.getZ() - self.line_thicknesses[cameras[1]][1]) ), 0.01)

    def drawPolygon2d(self, polygon, color, alpha, facecolor=None):
        # Draws a polygon with drawLine2d
        points = polygon.ToArray()
        # self.line_thickness = self.calcLineThickness()
        for point in points:
            self.generator.link_segment(
                Vec3(point[0], point[1], 2), Vec4(0, 0, 1, 1),
                self.line_thickness, self.getColor(color))
        self.generator.link_segment_end(Vec4(0, 0, 1, 1), self.getColor(color))


    # def drawLine2d(self, line2d, color=Viewer.Color.Blue, alpha=1.0, zorder=1, dashed=False):
    def drawLine2d(self, line2d, color='blue', alpha=1.0, dashed=False, zorder=1, linewidth=1):
        # TODO: enable dashed line
        line2d_np = line2d.ToArray()
        for point in line2d_np:
            self.generator.link_segment(
                Vec3(point[0], point[1], 2), Vec4(0, 0, 1, 1),
                self.line_thickness, self.getColor(color))
        self.generator.link_segment_end(Vec4(0, 0, 1, 1), self.getColor(color))
    
    def get_aspect_ratio(self):
      # ratio is width / height
      return 1.
    
    def drawAgent(self, agent, color, alpha, facecolor):
        """Draws an agent object with a model previosly set in set with self.agent_model_path
        Arguments:
            agent -- Agent object from world
        """
        # Adds new agent to dict of Panda3d nodes
        if not agent.id in self.agent_nodes:
            self.agent_nodes[agent.id] = self.loader.loadModel(
                self.agent_model_path)
            self.agent_nodes[agent.id].reparentTo(self.render)
            self.agent_nodes[agent.id].setPos(0, 0, 2)
            self.agent_nodes[agent.id].setScale(
                self.agent_scale[0], self.agent_scale[1], self.agent_scale[2])
            self.agent_poses[agent.id] = np.array([0, 0, 0], dtype=float)

            self.camera_list.insert(len(self.camera_list), agent.id)

        # Set new pose of agent in world cordinates
        state = agent.followed_trajectory[-1, :]
        self.agent_poses[agent.id][0] = state[int(StateDefinition.X_POSITION)]
        self.agent_poses[agent.id][1] = state[int(StateDefinition.Y_POSITION)]
        self.agent_poses[agent.id][2] = state[int(
            StateDefinition.THETA_POSITION)]
        transformed_polygon = agent.shape.Transform(self.agent_poses[agent.id])
        # self.drawPolygon2d(transformed_polygon, self.getColor(Viewer.Color.Blue), 1.0)

        # Fitting of 3d-Model frame to agent positon
        angle = ((self.agent_poses[agent.id][2] * 180) / pi +
                 self.agent_orientation[0])
                 
        self.agent_nodes[agent.id].setHpr(angle, self.agent_orientation[1],
                                          self.agent_orientation[2])
        
        translation = self.agent_translation
        if not np.all(translation == 0):
            r = np.array([[
                np.cos(self.agent_poses[agent.id][2]),
                np.sin(self.agent_poses[agent.id][2])
            ],
            [
                np.cos(self.agent_poses[agent.id][2] + pi * 0.5),
                np.sin(self.agent_poses[agent.id][2] + pi * 0.5)
            ]],
                         dtype=float)
            translation = np.dot(translation, r)
        self.agent_nodes[agent.id].setPos(
            self.agent_poses[agent.id][0] + translation[0],
            self.agent_poses[agent.id][1] + translation[1], 2)


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
        self.generator.begin(base.cam, self.render)
        super(Panda3dViewer, self).drawWorld(world, eval_agent_ids, filename, scenario_idx, debug_text)
        self.generator.end()
        self.taskMgr.step()

    def Reset(self):
        # for key, agent in self.agent_nodes.items():
        #   agent.removeNode()
        # self.agent_nodes = {}
        # self.follow_agent_id = 
        for an in self.agent_nodes.values():
          an.removeNode()
        self.agent_poses = {}
