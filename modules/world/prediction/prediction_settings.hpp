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


class PredictionSettings {
  public:
    PredictionSettings() : ego_prediction_model_(), others_prediction_model_() {}
    PredictionSettings(const BehaviorModelPtr& ego_prediction, const BehaviorModelPtr& others_prediction);
    virtual ~PredictionSettings() {}

    void ApplySettings(ObservedWorld& observed_world) const;

  private:
    BehaviorModelPtr ego_prediction_model_;
    BehaviorModelPtr others_prediction_model_;
};



}  // namespace prediction
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_PREDICTION_HPP_