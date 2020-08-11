// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/world/prediction/prediction_settings.hpp"
#include "bark/world/observed_world.hpp"
#include "bark/models/behavior/not_started/behavior_not_started.hpp"
// #include "bark/models/behavior/constant_velocity/constant_velocity.hpp"

namespace bark {
namespace world {
namespace prediction {

using bark::models::behavior::BehaviorModelPtr;

PredictionSettings::PredictionSettings(
    const BehaviorModelPtr& ego_prediction_model,
    const BehaviorModelPtr& default_prediction_model,
    const BehaviorModelPtr& specific_prediction_model,
    const std::vector<AgentId>& specific_prediction_agents)
    : ego_prediction_model_(ego_prediction_model),
      specific_prediction_model_(specific_prediction_model),
      default_prediction_model_(default_prediction_model),
      specific_prediction_agents_(specific_prediction_agents.begin(),
                                  specific_prediction_agents.end()) {}

void PredictionSettings::ApplySettings(
    bark::world::ObservedWorld& observed_world) const {
  observed_world.SetEgoBehaviorModel(
      BehaviorModelPtr(ego_prediction_model_->Clone()));
    if (default_prediction_model_) {
    for (const auto& agent : observed_world.GetOtherAgents()) {
      if (agent.second->GetBehaviorStatus() == models::behavior::BehaviorStatus::VALID) {
        if (specific_prediction_agents_.count(agent.first) == 0) {
            observed_world.SetBehaviorModel(
              agent.first, BehaviorModelPtr(default_prediction_model_->Clone()));
          // LOG(INFO) << "using " << agent.first << " as multi-agent";    
        }
      } else {
        // LOG(INFO) << "using BehaviorNotStarted for " << agent.first << " (not started yet)";
        observed_world.SetBehaviorModel(agent.first, std::make_shared<bark::models::behavior::BehaviorNotStarted>(nullptr));
      }
    }
  }
  if (specific_prediction_model_) {
    for (const auto& agent_id : specific_prediction_agents_) {
      // LOG(INFO) << "setting specific model for " << agent_id;
      observed_world.SetBehaviorModel(
          agent_id, BehaviorModelPtr(specific_prediction_model_->Clone()));
    }
  }
}
}  // namespace prediction
}  // namespace world
}  // namespace bark
