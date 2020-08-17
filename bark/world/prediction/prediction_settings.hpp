// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_PREDICTION_HPP_
#define BARK_WORLD_PREDICTION_HPP_

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

struct PredictionSettings {
  PredictionSettings() : ego_prediction_model_(), others_prediction_model_() {}
  PredictionSettings(const BehaviorModelPtr& ego_prediction,
                     const BehaviorModelPtr& others_prediction);
  virtual ~PredictionSettings() {}

  BehaviorModelPtr GetEgoPredictionModel() const {
    return ego_prediction_model_;
  }
  BehaviorModelPtr GetOthersPredictionModel() const {
    return others_prediction_model_;
  }

  void ApplySettings(ObservedWorld& observed_world) const;

  BehaviorModelPtr ego_prediction_model_;
  BehaviorModelPtr others_prediction_model_;
};

}  // namespace prediction
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_PREDICTION_HPP_