from modules.runtime.runtime import Runtime

import numpy as np
import tensorflow as tf

# tfa specific
from tf_agents.environments import py_environment
from tf_agents.specs import array_spec
from tf_agents.environments import wrappers
from tf_agents.trajectories import time_step as ts

tf.compat.v1.enable_v2_behavior()

class TFAWrapper(py_environment.PyEnvironment):
  """Wrapper for TensorFlow Agents (https://github.com/tensorflow/agents)
  
  Arguments:
    py_environment -- Base class for environment from tf_agents
  """

  def __init__(self, env):
    self.env = env
    # TODO(@hart): fill min and max values
    self._action_spec = array_spec.BoundedArraySpec(
        shape=self.env.action_space.shape,
        dtype=np.float32, minimum=-1.0, maximum=1.0, name='action')
    self._observation_spec = array_spec.BoundedArraySpec(
        shape=self.env.observation_space.shape,
        dtype=np.float32, minimum=-1000.0, maximum=1000.0, name='observation')
    self._state = np.zeros(shape=self.env.observation_space.shape,
        dtype=np.float32)
    self._episode_ended = False

  def action_spec(self):
    return self._action_spec

  def observation_spec(self):
    return self._observation_spec

  def render(self): 
    return self.env.render()
  
  def _reset(self):
    self._state = np.array(self.env.reset(), dtype=np.float32)
    self._episode_ended = False
    return ts.restart(self._state)

  def _step(self, action):
    if self._episode_ended:
      return self.reset()
    state, reward, self._episode_ended, _ = self.env.step(action)
    self._state = np.array(state, dtype=np.float32)
    if self._episode_ended:
      return ts.termination(self._state, reward=reward)
    else:
      return ts.transition(self._state, reward=reward, discount=1.0)



