import sys
import glob
import os
import math
import numpy as np

try:
  sys.path.append(glob.glob('tools/carla/carla-*.egg')[0])
except IndexError:
  pass

import carla


class CarlaClient():
  """docstring for CarlaParser"""

  def __init__(self, map='Town02'):
    self.client = None
    self.world = None
    self.bp_lib = None
    self.map = map
    # TODO: set/dict/list?
    self.vehicles = dict()

  def __del__(self):
    try:
      self.set_synchronous_mode(False)
    except Exception as e:
      pass

    if self.client != None:
      self.client.apply_batch([carla.command.DestroyActor(_) for _ in self.vehicles])

  def connect(self, host='localhost', port=2000, timeout=2):
    self.client = carla.Client(host, port)
    self.client.set_timeout(timeout)  # in second
    self.client.load_world(self.map)
    self.world = self.client.get_world()
    self.bp_lib = self.world.get_blueprint_library()

  def set_synchronous_mode(self, mode, delta_seconds=0.05):
    self.world.apply_settings(carla.WorldSettings(
        synchronous_mode=mode,
        fixed_delta_seconds=delta_seconds))

  def get_blueprint_library(self):
    return self.bp_lib

  def get_spawn_points(self):
    return self.world.get_map().get_spawn_points()

  def spawn_actor(self, blueprint, transform):
    try:
      actor = self.world.spawn_actor(blueprint, transform)
      self.vehicles[actor.id] = actor
      return actor.id
    except Exception as e:
      print(e)
      return None

  def spawn_sensor(self):
    pass

  def get_vehicles(self):
    return self.vehicles.keys()

  def set_autopilot(self, id, mode):
    self.vehicles[id].set_autopilot(mode)

  def get_vehicle_state(self, id):
    # Should be called in synchronous mode

    if id in self.vehicles:
      snapshot = self.world.get_snapshot()
      # vehicle = snapshot.find(id)
      vehicle = self.vehicles[id]

      t = vehicle.get_transform()
      v = vehicle.get_velocity()
      # c = vehicle.get_control()
      # a = vehicle.get_acceleration()
      # av = vehicle.get_angular_velocity()

      # [TIME_POSITION, X_POSITION, Y_POSITION, THETA_POSITION, VEL_POSITION, ...]
      return np.array([snapshot.timestamp.elapsed_seconds, t.location.x, -t.location.y,
                       math.radians(-t.rotation.yaw),
                       math.sqrt(v.x**2 + v.y**2 + v.z**2)])
    else:
      print("Actor {} not found".format(id))
      return None

  def get_all_vehicles_state(self, id_convertion):
    # id_convertion: convert carla actor id to bark agent id
    # Should be called in synchronous mode

    actor_state_map = dict()
    snapshot = self.world.get_snapshot()
    for id, vehicle in self.vehicles.items():

      t = vehicle.get_transform()
      v = vehicle.get_velocity()
      # c = vehicle.get_control()
      # a = vehicle.get_acceleration()
      # av = vehicle.get_angular_velocity()

      actor_state_map[id_convertion[id]] = np.array([snapshot.timestamp.elapsed_seconds, t.location.x, -t.location.y,
                                                     math.radians(-t.rotation.yaw),
                                                     math.sqrt(v.x**2 + v.y**2 + v.z**2)], dtype=np.float32)

    return actor_state_map

  def tick(self):
    pass
