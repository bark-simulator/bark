// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_PREDICTION_HPP_
#define BARK_WORLD_PREDICTION_HPP_

#include <set>
#include <typeinfo>
#include <unordered_map>

#include "bark/models/behavior/behavior_model.hpp"

namespace bark {
namespace world {
class ObservedWorld;
typedef std::shared_ptr<ObservedWorld> ObservedWorldPtr;

namespace prediction {

using models::behavior::BehaviorModelPtr;
using world::objects::AgentId;

/**
 * @brief Prediction settings allow to set behavior models for the ego vehicle,
 * a default prediction model for the other vehicle, and a specific prediction
 * model for some agent ids (can be used e.g. to use a more detailed model for
 * nearby agents)
 *
 */
struct PredictionSettings {
  PredictionSettings()
      : ego_prediction_model_(), specific_prediction_model_() {}
  PredictionSettings(const BehaviorModelPtr& ego_prediction_model,
                     const BehaviorModelPtr& default_prediction_model);
  PredictionSettings(const BehaviorModelPtr& ego_prediction_model,
                     const BehaviorModelPtr& default_prediction_model,
                     const BehaviorModelPtr& specific_prediction_model,
                     const std::vector<AgentId>& specific_prediction_agents);
  virtual ~PredictionSettings() {}

  BehaviorModelPtr GetEgoPredictionModel() const {
    return ego_prediction_model_;
  }
  BehaviorModelPtr GetOthersPredictionModel() const {
    return specific_prediction_model_;
  }

  void ApplySettings(ObservedWorld& observed_world) const;

  BehaviorModelPtr ego_prediction_model_;
  BehaviorModelPtr specific_prediction_model_;
  BehaviorModelPtr default_prediction_model_;
  std::set<AgentId> specific_prediction_agents_;
};

}  // namespace prediction
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_PREDICTION_HPP_