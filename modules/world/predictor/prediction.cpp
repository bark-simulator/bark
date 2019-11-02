#include "modules/world/predictor/prediction.hpp"
#include "modules/models/behavior/nop/nop.hpp"

namespace modules {
namespace world {
namespace prediction {

using goal_definition::GoalDefinition;
using goal_definition::GoalDefinitionPtr;
using models::behavior::BehaviorNOP;
using models::dynamic::StateDefinition;

Prediction::Prediction(commons::Params *params, const ObservedWorld &observed_world, const float time_step)
    : commons::BaseType(params), observed_world_(observed_world), time_step_(time_step) {
  // Create an AgentPrediction object for each other agent in the observed world
  for (auto const &agent : observed_world_.get_other_agents()) {
    AgentPrediction agent_prediction = {agent.first, agent.second->get_shape(), {}};
    agent_predictions_.insert(AgentPredictions::value_type(agent.first, agent_prediction));
  }
}

void Prediction::Predict(const uint n_steps, const std::map<AgentId, std::pair<BehaviorModelPtr, float>> &assumed_agent_behaviors) {
  ObservedWorldPtr prediction_world(observed_world_.Clone());
  BehaviorModelPtr ego_behavior = std::make_shared<BehaviorNOP>(get_params());
  prediction_world->set_ego_behavior_model(ego_behavior);

  HypothesisId hypothesis_id = 0;
  for (auto const &agent_behavior : assumed_agent_behaviors) {
    // Add agent with assumed behavior
    prediction_world->set_behavior_model(agent_behavior.first, agent_behavior.second.first);

    // Add a new MotionHypothesis object to agent_predictions with a new unique hypothesis_id
    if (!agent_predictions_.at(agent_behavior.first).motion_hypotheses_.empty()) {
      hypothesis_id = (--agent_predictions_.at(agent_behavior.first).motion_hypotheses_.end())->second.id + 1;
    }
    StochasticState initial_state = {
      prediction_world->get_other_agents().at(agent_behavior.first)->get_current_state(),
      StateCovariance::Identity()};
    MotionHypothesis new_hypothesis = {hypothesis_id, agent_behavior.second.second, {initial_state}};
    agent_predictions_.at(agent_behavior.first).motion_hypotheses_[hypothesis_id] = new_hypothesis;
  }

  // Predict for n_steps steps, add the agents' states at each time step to agent_predictions
  for (uint i = 1; i < n_steps; ++i) {
    prediction_world = prediction_world->Predict(time_step_);
    if (prediction_world->get_other_agents().empty()) {
      std::cout << "Error" << std::endl;
    }
    for (auto const &agent : prediction_world->get_other_agents()) {
      StochasticState predicted_state = {agent.second->get_current_state(), StateCovariance::Identity()};
      agent_predictions_.at(agent.first).motion_hypotheses_.at(hypothesis_id).states.push_back(predicted_state);
    }
  }
}

} // namespace prediction
} // namespace world
} // namespace modules
