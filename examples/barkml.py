from modules.runtime.commons.parameters import ParameterServer

from configurations.bark_agent import BARKMLBehaviorModel
from configurations.sac_highway.configuration import SACHighwayConfiguration


# TODO(@hart): actually the internal parameter server should be okay as the agent cannot be trained for anything else
# TODO(@hart): we might need to set the world boundaries differntly in the world than in the scenario used for training
# TODO(@hart): try the agent in a different scenario

params = ParameterServer(filename="examples/params/deterministic_scenario.json")
sac_configuration = SACHighwayConfiguration(params)

# bark_agent is of type behavior model
bark_agent = BARKAgent(configuration=sac_configuration)