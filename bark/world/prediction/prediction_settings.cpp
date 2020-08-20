// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/world/prediction/prediction_settings.hpp"
#include "bark/models/behavior/not_started/behavior_not_started.hpp"
#include "bark/world/observed_world.hpp"
// #include "bark/models/behavior/constant_velocity/constant_velocity.hpp"

namespace bark {
namespace world {
namespace prediction {

using bark::models::behavior::BehaviorModelPtr;
using bark::models::behavior::BehaviorNotStarted;
using models::behavior::BehaviorStatus;

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
  // TODO: Clean up HACK to keep Behavior Status
  auto ego_behavior_model_ptr =
      BehaviorModelPtr(ego_prediction_model_->Clone());
  ego_behavior_model_ptr->SetBehaviorStatus(
      observed_world.GetEgoBehaviorModel()->GetBehaviorStatus());
  observed_world.SetEgoBehaviorModel(ego_behavior_model_ptr);

  if (default_prediction_model_) {
    for (const auto& agent : observed_world.GetOtherAgents()) {
      if (agent.second->GetBehaviorStatus() == BehaviorStatus::VALID &&
          agent.second->IsValidAtTime(observed_world.GetWorldTime())) {
        if (specific_prediction_agents_.count(agent.first) == 0) {
          auto other_behavior_model_ptr =
              BehaviorModelPtr(default_prediction_model_->Clone());
          other_behavior_model_ptr->SetBehaviorStatus(
              default_prediction_model_->GetBehaviorStatus());
          observed_world.SetBehaviorModel(agent.first,
                                          other_behavior_model_ptr);
          // VLOG(1) << "using " << agent.first << " as multi-agent";
        }
      } else {
        // VLOG(1) << "using BehaviorNotStarted for " << agent.first << " (not
        // started yet)";
        observed_world.SetBehaviorModel(
            agent.first, std::make_shared<BehaviorNotStarted>(nullptr));
      }
    }
  }
  if (specific_prediction_model_) {
    for (const auto& agent_id : specific_prediction_agents_) {
      // LOG(INFO) << "setting specific model for " << agent_id;
      auto other_behavior_model_ptr =
          BehaviorModelPtr(specific_prediction_model_->Clone());
      other_behavior_model_ptr->SetBehaviorStatus(
          default_prediction_model_->GetBehaviorStatus());
      observed_world.SetBehaviorModel(agent_id, other_behavior_model_ptr);
    }
  }
}
}  // namespace prediction
}  // namespace world
}  // namespace bark
