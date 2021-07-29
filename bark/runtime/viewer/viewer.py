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
from bark.runtime.commons.parameters import ParameterServer
import math
from bark.core.world.evaluation.ltl import *
from bark.core.world.evaluation import *
from bark.core.models.behavior import *

logger = logging.getLogger()


class BaseViewer(Viewer):
    def __init__(self, params=None, **kwargs):
        if(params is None):
            params = ParameterServer()
        Viewer.__init__(self)
        self.initialize_params(params, kwargs)

    def initialize_params(self, params, kwargs={}):
        self.color_other_agents_line = params["Visualization"]["Agents"]["Color"]["Other"]["Lines",
                                                                                           "Color of other agents", (0.1, 0.1, 0.1)]
        self.color_other_agents_face = params["Visualization"]["Agents"]["Color"]["Other"]["Face",
                                                                                           "Color of other agents", (0.7, 0.7, 0.7)]
        self.color_eval_agents_line = params["Visualization"]["Agents"]["Color"]["Controlled"]["Lines",
                                                                                               "Color of controlled, evaluated agents", (0.9, 0, 0)]
        self.color_eval_agents_face = params["Visualization"]["Agents"]["Color"]["Controlled"]["Face",
                                                                                               "Color of controlled, evaluated agents", (0.9, 0, 0)]
        self.use_colormap_for_other_agents = params["Visualization"]["Agents"]["Color"][
            "UseColormapForOtherAgents", "Flag to enable color map for other agents", True]
        self.if_colormap_use_line_others = params["Visualization"]["Agents"]["Color"]["IfColormapUseLineColorOthers",
                                                                                      "Flag to enable that line color can be fixed for other agents while using colormap", True]
        self.alpha_eval_agent = params["Visualization"]["Agents"]["Alpha"]["Controlled",
                                                                           "Alpha of evalagents", 0.8]
        self.alpha_other_agents = params["Visualization"]["Agents"]["Alpha"]["Other",
                                                                             "Alpha of other agents", 1]
        self.route_color = params["Visualization"]["Agents"]["ColorRoute",
                                                             "Color of agents routes", (0.2, 0.2, 0.2)]
        self.draw_route = params["Visualization"]["Agents"]["DrawRoute",
                                                            "Draw Route of each agent", False]
        self.draw_agent_id = params["Visualization"]["Agents"]["DrawAgentId",
                                                               "Draw id of each agent", True]
        self.draw_invalid_agents = params["Visualization"]["Agents"]["DrawInvalidAgents", "Draw invalid agents with patch", False]
        self.draw_orientation_arrow = params["Visualization"]["Agents"]["DrawOrientationArrow",
                                                               "Draw Orientation of Arrow", False]
        self.draw_behavior_plan_eval_agent = params["Visualization"]["Agents"]["DrawBehaviorPlanEvalAgent", "Draw behavior plan of evalauted agent", False]
        self.draw_eval_goals = params["Visualization"]["Agents"]["DrawEvalGoals",
                                                                 "Draw Route of eval agent goals", True]
        self.eval_goal_color = params["Visualization"]["Agents"]["EvalGoalColor",
                                                                 "Color of eval agent goals", (.49, .63, .83)]
        self.draw_history = params["Visualization"]["Agents"]["DrawHistory",
                                                              "Draw history with alpha trace for each agent", False]
        self.draw_history_draw_face = params["Visualization"]["Agents"]["DrawHistoryDrawFace",
                                                                        "Flag to specify if face is drawn in history mode", True]

        # map
        self.color_lane_boundaries = params["Visualization"]["Map"]["XodrLanes"]["Boundaries"]["Color",
                                                                                               "Color of agents except ego vehicle", (0.7, 0.7, 0.7)]
        self.alpha_lane_boundaries = params["Visualization"]["Map"]["XodrLanes"][
            "Boundaries"]["Alpha", "Color of agents except ego vehicle", 1.0]
        self.plane_color = params["Visualization"]["Map"]["Plane"]["Color",
                                                                   "Color of the background plane", (1, 1, 1, 1)]
        self.plane_alpha = params["Visualization"]["Map"]["Plane"]["Alpha",
                                                                   "Alpha of the background plane", 1.0]
        self.map_linewidth = params["Visualization"]["Map"]["XodrLanes"]["Boundaries"]["Linewidth",
                                                                                       "Linewidth of linestrings", 1.0]
        self._draw_aerial_image = params["Visualization"]["Map"]["DrawAerialImage", "Flag to draw aerial image behind map", False]

        self.draw_ltl_debug_info = params["Visualization"]["Evaluation"]["DrawLTLDebugInfo",
                                                                         "Flag to specify if debug info to ltl evaluators shall be plotted", False]
                                                                        
        self.draw_rss_debug_info = params["Visualization"]["Evaluation"]["DrawRssDebugInfo",
                                                                         "Flag to specify if debug info to rss evaluators shall be plotted", False]

        self.draw_rss_safety_responses = params["Visualization"]["Evaluation"]["DrawRssSafetyResponses",
                                                                               "Flag to specify if visualizating rss safety responses.", False]

        self._draw_ego_rss_safety_responses = params["Visualization"]["Evaluation"][
          "DrawEgoRSSSafetyResponses",
          "Flag to specify if visualizating rss safety responses.",
          False]
        self._rss_min_braking_distances = params["Visualization"]["Evaluation"][
          "DrawMinRSSBrakingDistances",
          "Flag whether the min. braking distances shall be plotted.",
          False]
        
        self.parameters = params
        self.agent_color_map = {}
        self.max_agents_color_map = 0

        self.use_world_bounds = kwargs.pop("use_world_bounds", params["Visualization"]["Camera"]["UseWorldBounds", "", True])
        self.follow_agent_id = kwargs.pop("follow_agent_id", params["Visualization"]["Camera"]["FollowAgentIds", "", None])

        self.center = np.array(kwargs.pop("center", params["Visualization"]["Camera"]["Center", "", [0, 0]]))

        self.world_x_range = np.array(kwargs.pop("x_range", params["Visualization"]["Camera"]["XRange", "", [-40, 40]]))
        self.world_y_range = np.array(kwargs.pop("y_range", params["Visualization"]["Camera"]["YRange", "", [-40, 40]]))

        self.enforce_x_length = kwargs.pop("enforce_x_length", params["Visualization"]["Camera"]["EnforceXLength", "", True])
        self.enforce_y_length = kwargs.pop("enforce_y_length", params["Visualization"]["Camera"]["EnforceYLength", "", False])

        default_x_length = np.sum(np.absolute(self.world_x_range))
        default_x_length = np.sum(np.absolute(self.world_y_range))
        self.x_length = kwargs.pop(
            "x_length", params["Visualization"]["Camera"]["XLength", "", int(default_x_length)])
        self.y_length = kwargs.pop(
            "y_length", params["Visualization"]["Camera"]["YLength", "", int(default_x_length)])

        self.dynamic_world_x_range = self.world_x_range.copy()
        self.dynamic_world_y_range = self.world_y_range.copy()

    def reset(self):
        pass

    def get_aspect_ratio(self):
        pass

    def _get_draw_eval_agent_ids(self, world, eval_agent_ids=None, ):
        if self.follow_agent_id is not None:
            if isinstance(self.follow_agent_id, bool) and \
                    eval_agent_ids is not None and \
                    len(eval_agent_ids) == 1:
                draw_eval_agent_id = eval_agent_ids[0]
            else:
                draw_eval_agent_id = self.follow_agent_id

            if draw_eval_agent_id in world.agents:
                return draw_eval_agent_id

        return None

    def _update_world_view_range(self, world, eval_agent_ids=None):
        draw_eval_agent_id = self._get_draw_eval_agent_ids(
            world, eval_agent_ids)

        if draw_eval_agent_id != None:
            follow_agent = world.agents[draw_eval_agent_id]
            state = follow_agent.state
            pose = generatePoseFromState(state)

            center = [pose[0],  pose[1]]
            self._update_world_dynamic_range(center)
        else:
            if self.use_world_bounds:
                bb = world.bounding_box
                self.dynamic_world_x_range = [bb[0].x(), bb[1].x()]
                self.dynamic_world_y_range = [bb[0].y(), bb[1].y()]

                diffx = abs(
                    self.dynamic_world_x_range[1] - self.dynamic_world_x_range[0])
                diffy = abs(
                    self.dynamic_world_y_range[1] - self.dynamic_world_y_range[0])

                # enforce that in both dimensions  the same range is covered
                if diffx > diffy:
                    self.dynamic_world_y_range[0] -= (diffx - diffy)/2
                    self.dynamic_world_y_range[1] += (diffx - diffy)/2
                else:
                    self.dynamic_world_x_range[0] -= (diffy - diffx)/2
                    self.dynamic_world_x_range[1] += (diffy - diffx)/2
            else:
                center = self.center
                self._update_world_dynamic_range(center)

    def _update_world_dynamic_range(self, center):
        aspect_ratio = self.get_aspect_ratio()
        if self.enforce_x_length:
            self.dynamic_world_x_range = [-self.x_length /
                                          2 + center[0], self.x_length/2 + center[0]]
            self.dynamic_world_y_range = [-self.x_length/2/aspect_ratio +
                                          center[1], self.x_length/2/aspect_ratio + center[1]]

        if self.enforce_y_length:
            self.dynamic_world_x_range = [-self.y_length/2*aspect_ratio +
                                          center[0], self.y_length/2*aspect_ratio + center[0]]
            self.dynamic_world_y_range = [-self.y_length /
                                          2 + center[1], self.y_length/2 + center[1]]

    def drawPoint2d(self, point2d, color, alpha):
        pass

    def drawLine2d(self, line2d, color, alpha, line_style=None, zorder=2):
        pass

    def drawPolygon2d(self, polygon, color, alpha, facecolor=None, zorder=10, hatch=''):
        pass

    def drawTrajectory(self, trajectory, color, **kwargs):
        pass
    
    def drawArrow(self, pose):
        pass

    def drawObstacle(self, obstacle):
        pass

    def drawText(self, position, text, **kwargs):
        pass

    def drawCircles(self, position_list, radius):
        for pos in position_list:
          self.drawCircle(pos, radius)

    def drawCircle(self, position, radius):
        pass

    def getColor(self, color):
        pass

    def show(self, block=False):
        pass

    def clear(self):
        pass

    def drawAgents(self, world):
        for _, agent in world.agents.items():
            if agent.id in world.agents_valid:
              self.drawAgent(agent)
            else:
              if self.draw_invalid_agents:
                self.drawAgent(agent, hatch='o')
    
    def drawBehaviorPlan(self, agent):
        self.drawTrajectory(agent.behavior_model.last_trajectory,
                                  color='black', linewidth=1.0)

    def drawHistory(self, agent, color, alpha, facecolor, zorder):
        shape = agent.shape
        if isinstance(shape, Polygon2d):
            history = agent.history
            lh = len(history)
            for idx, state_action in enumerate(history):
                state = state_action[0]
                pose = generatePoseFromState(state)
                transformed_polygon = shape.Transform(pose)
                alpha = 1-0.8*(lh-idx)/3.4
                alpha = 0 if alpha < 0 else alpha
                # fade to 0.2 after 10 steps
                self.drawPolygon2d(transformed_polygon, color,
                                   alpha, facecolor, zorder=zorder)

    def drawGoalDefinition(self, goal_definition, color, alpha, facecolor):
        if isinstance(goal_definition, GoalDefinitionPolygon):
            self.drawPolygon2d(goal_definition.goal_shape,
                               color, alpha, facecolor, zorder=2)
        elif isinstance(goal_definition, GoalDefinitionStateLimits):
            self.drawPolygon2d(goal_definition.xy_limits,
                               color, alpha, facecolor, zorder=2)
        elif isinstance(goal_definition, GoalDefinitionStateLimitsFrenet):
            self.drawPolygon2d(goal_definition.goal_shape,
                               color, alpha, facecolor, zorder=2)
        elif isinstance(goal_definition, GoalDefinitionSequential):
            # draw only first goal
            self.drawGoalDefinition(goal_definition.sequential_goals[0], color,
                                alpha, facecolor)

    def drawLabelsAsText(self, observed_world, label_functions, evaluator_type):
        labels = {}
        for lf in label_functions:
            labels.update(lf.Evaluate(observed_world))
        str = ""
        for (k, v) in labels.items():
            # if k.agent_id in [-1, 104, 108]:
            if v:
                str = str + \
                    "{} {}_{} : {}\n".format(
                        evaluator_type, k.label_str, k.agent_id, v)
        self.drawText(position=(0.7, 0.9), text=str)

    def drawWorld(self, world, eval_agent_ids=None, filename=None, scenario_idx=None, debug_text=True):
      sensed_world = None
      if eval_agent_ids:
          sensed_world = world.agents[eval_agent_ids[0]].sensed_world
      if sensed_world:
          self.drawTrueWorld(world, eval_agent_ids)
          self.drawSensedWorld(sensed_world, eval_agent_ids, filename, scenario_idx, debug_text)
      else:
          self.drawWorldImplementation(world, eval_agent_ids, filename, scenario_idx, debug_text)

    def drawSensedWorld(self, sensed_world, eval_agent_ids=None, filename=None, scenario_idx=None, debug_text=True):
      self.drawWorldImplementation(sensed_world, eval_agent_ids, filename, scenario_idx, debug_text)

    def drawTrueWorld(self, world, eval_agent_ids):
      # draw only boundaries of agents in same color as observed agents
      for i, (agent_id, agent) in enumerate(world.agents.items()):
          color_line, color_face, alpha = self.GetAgentColor(agent, eval_agent_ids)
          color_face = (1.0, 1.0, 1.0) # face color white
          self.drawAgent(agent, color_line, alpha, color_face, hatch='', draw_agent_id=False)

    def GetAgentColor(self, agent, eval_agent_ids):
      alpha = 1.0
      agent_id = agent.id
      if eval_agent_ids and agent.id in eval_agent_ids:
          color_line = self.color_eval_agents_line
          color_face = self.color_eval_agents_face
          alpha = self.alpha_eval_agent
      else:
          alpha = self.alpha_other_agents
          if self.use_colormap_for_other_agents:
              if not agent_id in self.agent_color_map:
                  color_idx = len(self.agent_color_map) % self.getSizeOfColormap()
                  self.agent_color_map[agent_id] = self.getColorFromMap(color_idx)
              if self.if_colormap_use_line_others:
                  color_line = self.color_other_agents_line
              else:
                  color_line = self.agent_color_map[agent_id]
              color_face = self.agent_color_map[agent_id]
          else:
              color_line = self.color_other_agents_line
              color_face = self.color_other_agents_face
      return color_line, color_face, alpha

    def drawWorldImplementation(self, world, eval_agent_ids=None, filename=None, scenario_idx=None, debug_text=True):
        # self.clear()
        self._update_world_view_range(world, eval_agent_ids)
        if world.map:
            self.drawMap(world.map)

        if self._draw_aerial_image:
          self.drawMapAerialImage()

        # draw agent goals
        for agent_id, agent in world.agents.items():
            if eval_agent_ids and self.draw_eval_goals and agent.goal_definition and \
                    agent_id in eval_agent_ids:
                color_line = self.color_eval_agents_line
                color_face = self.color_eval_agents_face
                alpha = self.alpha_eval_agent
                self.drawGoalDefinition(
                    agent.goal_definition, color_line, alpha, color_face)

        num_agents = len(world.agents.items())
        for i, (agent_id, agent) in enumerate(world.agents.items()):
            color_line, color_face, alpha = self.GetAgentColor(agent, eval_agent_ids)
                    
            if self.draw_history and agent.id in world.agents_valid:
                if self.draw_history_draw_face:
                    color_face_history = color_face
                else:
                    color_face_history = (1.0, 1.0, 1, .0)
                self.drawHistory(agent, color_line, alpha,
                                 color_face_history, zorder=5)
            
            hatch = '' if agent.id in world.agents_valid else 'o'
            if agent.id in world.agents_valid or self.draw_invalid_agents:
              self.drawAgent(agent, color_line, alpha, color_face, hatch=hatch)
              
        if debug_text:
            self.drawText(position=(0.1, 0.9), text="Scenario: {}".format(
                scenario_idx), fontsize=14)
            self.drawText(position=(0.1, 0.95),
                          text="Time: {:.2f}".format(world.time), fontsize=14)

        if self.draw_ltl_debug_info:
            self.drawLTLDebugInfomation(world, eval_agent_ids[0])

        if self.draw_rss_debug_info or self.draw_rss_safety_responses:
            if self.draw_rss_debug_info:
                self.drawRssDebugInfomation(world, eval_agent_ids[0])
            if self.draw_rss_safety_responses:
                self.drawRssSafetyResponses(world, eval_agent_ids[0])

        if self.draw_behavior_plan_eval_agent:
          eval_agent = world.GetAgent(eval_agent_ids[0])
          if eval_agent is not None:
              self.drawBehaviorPlan(eval_agent)
        
        if self._draw_ego_rss_safety_responses:
          self.DrawRSSEvaluatorState(world, eval_agent_ids[0])
        
        if self._rss_min_braking_distances:
          self.DrawRSSBrakingDistances(world, eval_agent_ids[0])

    def drawMapAerialImage(self):
        pass

    def drawMap(self, map_interface):
        # draw the boundary of each lane
        road_graph = map_interface.GetRoadgraph()
        for lane_id in road_graph.GetAllLaneids():
            self.drawLanePolygon(map_interface.GetLane(lane_id), map_interface, self.color_lane_boundaries)

        for _, road in map_interface.GetOpenDriveMap().GetRoads().items():
            self.drawXodrRoad(road, self.color_lane_boundaries)

    def drawLanePolygon(self, lane, map_interface, color=None):
        if color is None:
            self.color_lane_boundaries
        
        polygon = map_interface.GetRoadgraph().GetLanePolygonForLaneId(lane.lane_id)
        if not lane.lane_type == XodrLaneType.driving:
          self.drawPolygon2d(polygon, ( 0.5, 0.5 , 0.5),
                                1.0, ( 0.5, 0.5 , 0.5), linewidth=0.02,  zorder=1)
        else:
          self.drawPolygon2d(polygon, (0.7, 0.7, 0.7),
                                1.0, (0.7, 0.7, 0.7), linewidth=0.02,  zorder=1, hatch="/")

    def drawXodrRoad(self, road, color=None):
        for lane_section in road.lane_sections:
            self.drawXodrLaneSection(lane_section, color)

    def drawXodrLaneSection(self, lane_section, color=None):
        for _, lane in lane_section.GetLanes().items():
            self.drawXodrLane(lane, color)

    def drawXodrLane(self, lane, color=None):
        if color is None:
            self.color_lane_boundaries

        dashed = False
        color = ( 0.5, 0.5 , 0.5)
        # center line is type none and is drawn as broken
        if lane.road_mark.type == XodrRoadMarkType.broken:
            dashed = True
            color = (1, 1 ,1)
 
        if not lane.road_mark.type == XodrRoadMarkType.none:
          self.drawLine2d(lane.line, color, self.alpha_lane_boundaries,
                          dashed, zorder=1, linewidth=1)

    def drawAgent(self, agent, color, alpha, facecolor, hatch='', draw_agent_id=True):
        shape = agent.shape
        if isinstance(shape, Polygon2d):
            state = agent.state
            pose = generatePoseFromState(state)
            transformed_polygon = shape.Transform(pose)

            centerx = (shape.front_dist - 0.5*(shape.front_dist +
                                               shape.rear_dist)) * math.cos(pose[2]) + pose[0]
            centery = (shape.front_dist - 0.5*(shape.front_dist +
                                               shape.rear_dist)) * math.sin(pose[2]) + pose[1]

            if self.draw_agent_id:
                angle = min(pose[2], pose[2] - math.pi / 2.0)
                self.drawText(position=(centerx, centery), rotation=180.0*(angle/math.pi), text="{}".format(agent.id),
                              coordinate="not axes", ha='center', va="center", multialignment="center", size="smaller")
            
            if self.draw_orientation_arrow:
              self.drawArrow(pose)

            self.drawPolygon2d(transformed_polygon, color,
                               alpha, facecolor, zorder=10, hatch=hatch)
        else:
            raise NotImplementedError("Shape drawing not implemented.")

    def drawLaneCorridor(self, lane_corridor, color=None):
        if color is None:
            color = "blue"
        self.drawPolygon2d(lane_corridor.polygon, color=color,
                           facecolor=color, alpha=.3, zorder=2)
        self.drawLine2d(lane_corridor.center_line, "k")

    def Reset(self):
        pass

    def drawRoadCorridor(self, road_corridor, color=None):
        if color is None:
            color = "blue"
        self.drawPolygon2d(road_corridor.polygon, color,
                           facecolor=color, alpha=.2, zorder=2)

    def drawLTLDebugInfomation(self, world, agent_id):
        observed_world = world.Observe([agent_id])
        if len(observed_world) == 0:
            return
        observed_world = observed_world[0]
        for _, evaluator_type in enumerate(observed_world.evaluators):
            if isinstance(observed_world.evaluators[evaluator_type], EvaluatorLTL):
                label_functions = observed_world.evaluators[evaluator_type].label_functions
                # we will only plot labels of first ltl evaluator
                self.drawLabelsAsText(
                    observed_world, label_functions, evaluator_type)
                break

    def drawEvalResults(self, eval_results):
        y_pos = 0.9
        for eval_name, eval_result in eval_results.items():
          text = "{}: {}".format(eval_name, eval_result)
          self.drawText(position=(0.7, y_pos), text=text)
          y_pos -= 0.03

    def drawRssDebugInfomation(self, world, agent_id):
        from bark.core.world.evaluation import EvaluatorRSS
        for evaluator in world.evaluators:
          if isinstance(world.evaluators[evaluator], EvaluatorRSS):
            rss_responses = world.evaluators[evaluator].PairwiseDirectionalEvaluate(
                world)
            break

        def char_func(value):
            if value == True:
                return "T"
            elif value == False:
                return "F"
            else:
                return "UNKNOWN"

        overall_safety = True
        if rss_responses:
            self.drawText(position=(0.82, 0.91), text="ID  Lon  Lat")
            for i, (id, responses) in enumerate(rss_responses.items()):
                overall_safety = overall_safety and any(responses)
                str = "{}:    {}     {}".format(
                    id, *list(map(char_func, responses)))
                self.drawText(position=(0.82, 0.88-0.03*i), text=str)

        self.drawText(position=(0.74, 0.96), horizontalalignment="left", text="ego id {} safety: {}".format(
            agent_id, char_func(overall_safety)))
    
    def drawRssSafetyResponses(self, world, ego_id):
        from bark.core.world.evaluation import EvaluatorRSS
        for evaluator in world.evaluators:
            if isinstance(world.evaluators[evaluator], EvaluatorRSS):
                rss_responses = world.evaluators[evaluator].PairwiseEvaluate(
                  world)
                break

        ego_agent = world.agents[ego_id]
        shape = ego_agent.shape
        pose = generatePoseFromState(ego_agent.state)
        transformed_polygon = shape.ScalingTransform(1.5, pose)
        self.drawPolygon2d(
            transformed_polygon,
            self.color_eval_agents_line, 0.6, self.color_other_agents_face, linewidth=1.5, zorder=9)

        # draw response for other agents
        relevant_agents = [
            agent for agent in world.agents.values() if agent.id in rss_responses]
        for agent in relevant_agents:
            shape = agent.shape
            pose = generatePoseFromState(agent.state)
            transformed_polygon = shape.ScalingTransform(1.5, pose)

            response_color = "LightGreen" if rss_responses[agent.id] else "Red"
            self.drawPolygon2d(transformed_polygon, response_color,
                               0.6, response_color, zorder=9)

    def DrawRSSBrakingDistances(self, world, eval_id):
      for agent_id, agent in world.agents.items():
        observed_world = world.Observe([agent_id])[0]
        rss_params = None
        if eval_id == agent_id:
          rss_params = self.parameters["EvaluatorRss"]["Ego"]
        else:
          rss_params = self.parameters["EvaluatorRss"]["Others"]
        try:
          min_braking_safety_polygon = ComputeMinBrakingPolygon(observed_world, rss_params)
        except:
          pass
        self.drawPolygon2d(
          min_braking_safety_polygon.GetPolygon(), "black", 0.1, "gray", zorder=12)
        
    def DrawRSSEvaluatorState(self, world, agent_id):
      agent = world.agents[agent_id]
      behavior = agent.behavior_model

      if isinstance(behavior, BehaviorRSSConformant):
        longitudinal_response = behavior.GetLongitudinalResponse()
        lateral_left_response = behavior.GetLateralLeftResponse()
        lateral_right_response = behavior.GetLateralRightResponse()
        violations = []
        if longitudinal_response != 0:
          print("Longitudinal violation")
          violations.append({"label": "LON", "color": "purple"})
        if lateral_left_response != 0 or lateral_right_response != 0:
          print("Lateral violation")
          violations.append({"label": "LAT", "color": "red"})
        
        # have to compute shape here
        observed_world = world.Observe([agent_id])[0]
        behavior.ComputeSafetyPolygons(observed_world)
        safety_polygons = behavior.GetSafetyPolygons()
    
        for poly in safety_polygons:
          # print(poly)
          color_face = self.agent_color_map[poly.GetAgentId()]
          self.drawPolygon2d(poly.GetPolygon(), color_face, 0.78, color_face, zorder=9)
          
        # draw labels
        ego_agent = world.agents[agent_id]
        shape = ego_agent.shape
        pose = generatePoseFromState(ego_agent.state)
        a = shape.front_dist - 0.5*(shape.front_dist + shape.rear_dist)
        centerx = a * math.cos(pose[2]) + pose[0] + 2.5
        centery = a * math.sin(pose[2]) + pose[1] + shape.right_dist + 2.
        
        if self.draw_agent_id:
          for violation in violations:
            t = self.drawText(
              position=(centerx, centery), rotation=180.0*(1.0+pose[2]/math.pi),
              text="{}".format(violation["label"]),
              coordinate="not axes", ha='center', va="center",
              multialignment="center", size="smaller",
              color = violation["color"])
            centerx += 5.
            t.set_bbox(dict(
              facecolor=violation["color"], alpha=0.5,
              edgecolor=violation["color"]))


def generatePoseFromState(state):
  # pybind creates column based vectors, initialization maybe row-based -> we consider both
  pose = np.zeros(3)
  pose[0] = state[int(StateDefinition.X_POSITION)]
  pose[1] = state[int(StateDefinition.Y_POSITION)]
  pose[2] = state[int(StateDefinition.THETA_POSITION)]
  return pose
