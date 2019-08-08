import tensorflow as tf
tf.compat.v1.enable_v2_behavior()
from tf_agents.drivers import dynamic_step_driver
from tf_agents.utils import common
from modules.runtime.ml.runners.base_runner import BaseRunner

class TFARunner(BaseRunner):
  def __init__(self,
                runtime,
                agent,
                number_of_collections=10,
                initial_collection_steps=10,
                collection_steps_per_cycle=10,
                initial_collection_driver=None,
                collection_driver=None):
    BaseRunner.__init__(self,
                        runtime=runtime,
                        agent=agent,
                        number_of_collections=number_of_collections,
                        initial_collection_steps=initial_collection_steps,
                        collection_steps_per_cycle=collection_steps_per_cycle)
    self.get_initial_collection_driver()
    self.get_collection_driver()
  
  def get_initial_collection_driver(self):
    self._initial_collection_driver = dynamic_step_driver.DynamicStepDriver(
      self._runtime,
      self._agent._agent.collect_policy,
      observers=[self._agent._replay_buffer.add_batch],
      num_steps=self._initial_collection_steps)
    self._initial_collection_driver.run = common.function(self._initial_collection_driver.run)

  def get_collection_driver(self):
    self._collection_driver = dynamic_step_driver.DynamicStepDriver(
      self._runtime,
      self._agent._agent.collect_policy,
      observers=[self._agent._replay_buffer.add_batch],
      num_steps=self._collection_steps_per_cycle)
    self._collection_driver.run = common.function(self._collection_driver.run)
   
  def collect_initial_episodes(self):
    self._initial_collection_driver.run()
                      
  def train(self):
    iterator = iter(self._agent._dataset)
    for i in range(0, self._number_of_collections):
      print("Collecting {}".format(str(i)))
      self._collection_driver.run()

      # train
      experience, _ = next(iterator)
      train_loss = self._agent._agent.train(experience)
      print('train_loss = {}'.format(train_loss.loss))


  def evaluate(self, render=False):
    # for loop
    # cannot use collection driver due to the fact it would evaluate the whole dataset
    # greedy policy
    # eval metrics
    pass
