// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/world/prediction/prediction_settings.hpp"
#include "bark/world/observed_world.hpp"

namespace bark {
namespace world {
namespace prediction {

using bark::models::behavior::BehaviorModelPtr;

PredictionSettings::PredictionSettings(
    const BehaviorModelPtr& ego_prediction,
    const BehaviorModelPtr& others_prediction)
    : ego_prediction_model_(ego_prediction),
      others_prediction_model_(others_prediction) {}

void PredictionSettings::ApplySettings(
    bark::world::ObservedWorld& observed_world) const {
  observed_world.SetEgoBehaviorModel(
      BehaviorModelPtr(ego_prediction_model_->Clone()));
  for (const auto& agent : observed_world.GetOtherAgents()) {
    observed_world.SetBehaviorModel(
        agent.first, BehaviorModelPtr(others_prediction_model_->Clone()));
  }
}

}  // namespace prediction
}  // namespace world
}  // namespace bark
