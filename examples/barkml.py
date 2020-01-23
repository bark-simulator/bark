from modules.runtime.commons.parameters import ParameterServer

from configurations.bark_agent import BARKMLBehaviorModel
from configurations.sac_highway.configuration import SACHighwayConfiguration


params = ParameterServer(filename="examples/params/deterministic_scenario.json")

# NOTE(@all): this won't work as a test since we need to have writing access
#             to the summaries folder in bark-ml

# bark_agent is of type behavior model (can be a trained ml agent)
# sac_configuration = SACHighwayConfiguration(params)
# bark_agent = BARKMLBehaviorModel(configuration=sac_configuration)
