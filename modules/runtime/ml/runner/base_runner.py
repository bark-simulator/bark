from abc import ABC, abstractmethod

class BaseRunner(ABC):
  def __init__(self,
               runtime,
               agent,
               number_of_collections=10,
               initial_collection_steps=0,
               collection_steps_per_cycle=1,
               train_metrics=None,
               eval_metrics=None,
               render_evaluation=True):
    self._runtime = runtime
    self._agent = agent
    self._number_of_collections = number_of_collections
    self._initial_collection_steps = initial_collection_steps
    self._collection_steps_per_cycle = collection_steps_per_cycle
    self._train_metric = train_metrics
    self._eval_metrics = eval_metrics
    self._render_evaluation = render_evaluation
    self.initial_episode_collection()

  @abstractmethod
  def _collect_initial_episodes(self, num):
    pass

  def initial_episode_collection(self):
    self._collect_initial_episodes(self._initial_collection_steps)

  @abstractmethod
  def train(self, render=False):
    pass

  @abstractmethod
  def evaluate(self, render=False):
    pass

  def render(self):
    if self._render_evaluation:
      self._runtime.render()

  def reset(self):
    self._runtime.reset()
    self._agent.reset()