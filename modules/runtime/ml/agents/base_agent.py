from abc import ABC, abstractmethod

class BaseAgent(ABC):
  def __init__(self,
               agent,
               replay_buffer=None,
               checkpointer=None,
               dataset=None):
    self._agent = agent
    self._replay_buffer = replay_buffer
    self._checkpointer = checkpointer
    self._dataset = dataset

  @abstractmethod
  def get_agent(self):
    pass

  @abstractmethod
  def get_replay_buffer(self):
    pass

  @abstractmethod
  def get_checkpointer(self):
    pass

  @abstractmethod
  def get_dataset(self):
    pass

  @abstractmethod
  def step(self, state):
    pass

  @abstractmethod
  def reset(self):
    pass