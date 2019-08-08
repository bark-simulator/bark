class SACAgent(BaseAgent):
    def __init__(self):
      self._agent = self.get_agent()
      self._replay_buffer = self.get_replay_buffer()
      self._checkpointer = self.get_checkpointer()
      self._dataset = self.get_dataset()
      BaseAgent.__init__(self,
                         agent=self._agent,
                         replay_buffer=self._replay_buffer,
                         checkpointer=self._checkpointer,
                         dataset=self._dataset)
  
    def get_agent(self):
      pass
  
    def get_replay_buffer(self):
      pass
  
    def get_checkpointer(self):
      pass
  
    def get_dataset(self):
      pass
  
    def step(self, state):
      pass
  
    def reset(self):
      self._agent.reset()