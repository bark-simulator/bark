
class TFARunner(BaseRunner):
    def __init__(self,
                 runtime,
                 agent,
                 initial_collection_steps=0,
                 collection_steps_per_cycle=1,
                 initial_collection_driver=None,
                 collection_driver=None):
      self._initial_collection_driver = self.get_initial_collection_driver()
      self._collection_driver = self.get_collection_driver()
      BaseRunner.__init__(self,
                          runtime=runtime,
                          agent=agent,
                          initial_collection_steps=initial_collection_steps,
                          collection_steps_per_cycle=collection_steps_per_cycle)
    
    def get_initial_collection_driver(self):
      pass
  
    def get_collection_driver(self):
      pass
  
    def _collect_initial_episodes(self):
      # self._agent .. initial collection driver
      pass
                        
    def train(self):
      # for loop
      # self._agent .. initial collection driver
      # train metrics
      pass
  
    def evaluate(self, render=False):
      # for loop
      # cannot use collection driver due to the fact it would evaluate the whole dataset
      # greedy policy
      # eval metrics
      pass
  