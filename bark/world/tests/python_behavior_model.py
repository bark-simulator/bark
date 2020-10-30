# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import unittest
import numpy as np
import time
import copy
import os

from bark.runtime.commons import ParameterServer
from bark.core.models.dynamic import StateDefinition

from bark.core.models.behavior import * 

# Simple python behavior model for testing purposes
class PythonDistanceBehavior(BehaviorModel):
  """Python behavior model to give an example of observed world interfaces
  """
  def __init__(self, params = None):
    super(PythonDistanceBehavior, self).__init__(params)
    self._params = params
    self._distance_range_constant_acceleration = \
            self._params["PythonDistanceBehavior::RangeConstantVelocity", \
          "Range in meters defining when controlled vehicle does not change velocity", [10, 20]]

  def Plan(self, delta_time, observed_world):
    # Get state of observer
    ego_agent = observed_world.ego_agent
    ego_agent_state = observed_world.ego_state # or via ego_agent.state
    ego_velocity = ego_agent_state[int(StateDefinition.VEL_POSITION)]
    ego_x = ego_agent_state[int(StateDefinition.X_POSITION)]
    ego_y = ego_agent_state[int(StateDefinition.Y_POSITION)]
    ego_theta = ego_agent_state[int(StateDefinition.THETA_POSITION)]
    print("Ego x: {}, y: {}, v: {}, theta: {}".format(
        ego_x, ego_y, ego_velocity, ego_theta))

    # get distance and state of observer's leading vehicle
    # 1) get tuple with agent and frenet position
    leading_agent_pair = observed_world.GetAgentInFront() # to get agent behind use GetAgentBehind()
    
    # 2) get state of leading agent from tuple
    leading_agent = leading_agent_pair[0]
    leading_agent_state = leading_agent.state
    leading_agent_x = leading_agent_state[int(StateDefinition.X_POSITION)]
    # to get other state values, same procedure as for ego agent
    # ....
    # 3) get frenet longitudinal distance to leading agent from tuple
    leading_frenet = leading_agent_pair[1]
    longitudinal_dist = leading_frenet.lon
    # 4) subtract vehicle shapes rear front to get true physical distance
    vehicle_margins = ego_agent.shape.front_dist + leading_agent.shape.rear_dist
    longitudinal_dist_margins = longitudinal_dist - vehicle_margins

    print("Leading x: {}, ... long dist: {}, long dist margins: {}".format(
        leading_agent_x, longitudinal_dist, longitudinal_dist_margins))

    # select some action 
    # we choose a very simple heuristic to select among actions 
    # decelerate, constant velocity or accelerate
    acceleration = None
    if longitudinal_dist_margins < self._distance_range_constant_acceleration[0]:
      acceleration = -4.0
    elif longitudinal_dist_margins > self._distance_range_constant_acceleration[0] and \
          longitudinal_dist_margins < self._distance_range_constant_acceleration[1]:
      acceleration = 0.0
    else:
      acceleration = 4.0

    # a motion primitive model converts it to trajectory
    behavior = PrimitiveConstAccStayLane(self._params, acceleration)
    traj = behavior.Plan(delta_time, observed_world, observed_world.lane_corridor)

    # set internal behavior parameters
    super(PythonDistanceBehavior, self).SetLastTrajectory(traj)
    super(PythonDistanceBehavior, self).SetLastAction(acceleration)
    print("Trajectory: {}".format(traj))
    return traj

  def Clone(self):
    return self

  def __setstate__(self, d):
    self._distance_range_constant_acceleration = d["range_constant_params"]

  def __getstate__(self):
    return {"range_constant_params": self._distance_range_constant_acceleration}