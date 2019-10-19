#include "modules/world/predictor/prediction.hpp"

namespace modules {
namespace world {
namespace prediction {

using goal_definition::GoalDefinition;
using goal_definition::GoalDefinitionPtr;

Prediction::Prediction(commons::Params *params, const ObservedWorld &observed_world, const float time_step)
    : commons::BaseType(params), observed_world_(observed_world), time_step_(time_step) {
  AgentMap other_agents = observed_world_.get_other_agents();
  observed_world_.clear_agents();

  for (auto const &agent : other_agents) {
    observed_world_.add_agent(agent.second);
    AgentPrediction agent_prediction = {agent.first, agent.second->get_shape(), {}};
    agent_predictions_.insert(AgentPredictions::value_type(agent.first, agent_prediction));
  }
}

void Prediction::Predict(const uint n_steps, const std::map<AgentId, std::pair<BehaviorModelPtr, float>> &assumed_agent_behaviors) {
  HypothesisId hypothesis_id = 0;
  for (auto const &agent_behavior : assumed_agent_behaviors) {
    observed_world_.set_behavior_model(agent_behavior.first, agent_behavior.second.first);

    if (!agent_predictions_.at(agent_behavior.first).motion_hypotheses_.empty()) {
      hypothesis_id = (--agent_predictions_.at(agent_behavior.first).motion_hypotheses_.end())->second.id + 1;
    }
    MotionHypothesis new_hypothesis = {hypothesis_id, agent_behavior.second.second, {}};
    agent_predictions_.at(agent_behavior.first).motion_hypotheses_[hypothesis_id] = new_hypothesis;
  }

  for (uint i = 0; i < n_steps; ++i) {
    ObservedWorldPtr prediction_world = observed_world_.Predict(i * time_step_);
    for (auto const &agent : prediction_world->get_other_agents()) {
      StochasticState predicted_state = {agent.second->get_current_state(), StateCovariance::Identity()};
      agent_predictions_.at(agent.first).motion_hypotheses_.at(hypothesis_id).states.push_back(predicted_state);
    }
  }
}

} // namespace prediction
} // namespace world
} // namespace modules
