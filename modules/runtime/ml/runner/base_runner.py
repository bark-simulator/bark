from abc import ABC, abstractmethod

class BaseRunner(ABC):
  def __init__(self,
               runtime,
               agent,
               initial_collection_steps=0,
               collection_steps_per_cycle=1,
               train_metrics=None,
               eval_metrics=None,
               render_evaluation=True):
    self._runtime = runtime
    self._agent = agent
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
  def train(self):
    pass

  @abstractmethod
  def evaluate(self):
    pass

  @abstractmethod
  def render(self):
    if self._render_evaluation:
      self._runtime.render()


class TFARunner(BaseRunner):
  def __init__(self,
               runtime,
               agent,
               initial_collection_steps=0,
               collection_steps_per_cycle=1):
    BaseRunner.__init__(runtime=runtime,
                        agent=agent,
                        initial_collection_steps=initial_collection_steps,
                        collection_steps_per_cycle=collection_steps_per_cycle)

  def _collect_initial_episodes(self):
    # self._agent .. initial collection driver
    pass
                      
  @abstractmethod
  def train(self):
    # for loop
    # self._agent .. initial collection driver
    # train metrics
    pass

  @abstractmethod
  def evaluate(self):
    # for loop
    # cannot use collection driver due to the fact it would evaluate the whole dataset
    # greedy policy
    # eval metrics
    pass