#include "modules/world/prediction/prediction.hpp"

namespace modules {
namespace world {
namespace prediction {

Prediction::Prediction(commons::Params *params, const ObservedWorld &observed_world, const std::map<AgentId, BehaviorModelPtr> &assumed_agent_behaviors)
    : commons::BaseType(params), observed_world_(observed_world) {
  // TODO(@AKreutz): Change behavior of agents that should not be observed
  for (auto const &agent : observed_world.get_other_agents()) {
    agent.second->set_behavior_model(assumed_agent_behaviors.at(agent.first));
  }
}

void Prediction::Step(const float time_step) {
  observed_world_.Step(time_step);
}

StochasticState Prediction::PredictAgentState(const AgentId agent_id) {
  State agent_state;
  for (auto const &agent : observed_world_.get_other_agents()) {
    if (agent.first == agent_id) {
      agent_state = agent.second->get_current_state();
      break;
    }
  }
  return {agent_state, StateCovariance::Identity()};
}

} // namespace prediction
} // namespace world
} // namespace modules