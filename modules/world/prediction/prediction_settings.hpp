// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_WORLD_PREDICTION_HPP_
#define MODULES_WORLD_PREDICTION_HPP_

#include <unordered_map>
#include <typeinfo>

#include "modules/models/behavior/behavior_model.hpp"

namespace modules {
namespace world {
class ObservedWorld;
typedef std::shared_ptr<ObservedWorld> ObservedWorldPtr;

namespace prediction {

using world::objects::AgentId;
using models::behavior::BehaviorModelPtr;


struct PredictionSettings {
  PredictionSettings() : ego_prediction_model_(), others_prediction_model_() {}
  PredictionSettings(const BehaviorModelPtr& ego_prediction, const BehaviorModelPtr& others_prediction);
  virtual ~PredictionSettings() {}

  BehaviorModelPtr get_ego_prediction_model() const { return ego_prediction_model_;}
  BehaviorModelPtr get_others_prediction_model() const { return others_prediction_model_;}

  void ApplySettings(ObservedWorld& observed_world) const;

  BehaviorModelPtr ego_prediction_model_;
  BehaviorModelPtr others_prediction_model_;
};



}  // namespace prediction
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_PREDICTION_HPP_