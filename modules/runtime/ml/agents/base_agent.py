from abc import ABC, abstractmethod

class BaseAgent(ABC):
  def __init__(self,
               agent,
               replay_buffer=None,
               collection_driver=None,
               checkpointer=None,
               dataset=None):
    self._agent = agent
    self._replay_buffer = replay_buffer
    self._collection_drivers = collection_driver
    self._checkpointer = checkpointer
    self._dataset = dataset

  @abstractmethod
  def step(self, state):
    pass

  @abstractmethod
  def reset(self, state):
    pass