// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_MODELS_BEHAVIOR_DYNAMIC_MODEL_HPP_
#define MODULES_MODELS_BEHAVIOR_DYNAMIC_MODEL_HPP_

#include "modules/models/behavior/behavior_model.hpp"
#include "modules/models/dynamic/dynamic_model.hpp"
#include "modules/world/world.hpp"

namespace modules {
namespace models {
namespace behavior {

using dynamic::Trajectory;
using dynamic::State;
using dynamic::DynamicModelPtr;
using dynamic::Input;
using world::objects::AgentId;
using world::ObservedWorld;

class DynamicBehaviorModel : public BehaviorModel {
 public:
  DynamicBehaviorModel(const DynamicModelPtr& dynamic_model,
                       commons::Params *params);

  DynamicBehaviorModel(DynamicBehaviorModel* other_behavior);
  virtual ~DynamicBehaviorModel() {}

  Trajectory Plan(float delta_time,
                  const ObservedWorld& observed_world);

  std::shared_ptr<BehaviorModel> Clone() const;

 private:
  DynamicModelPtr dynamic_model_;

  // Parameters
  float integration_time_delta_;
};

inline std::shared_ptr<BehaviorModel> DynamicBehaviorModel::Clone() const {
  std::shared_ptr<DynamicBehaviorModel> model_ptr =
    std::make_shared<DynamicBehaviorModel>(*this);
  return std::dynamic_pointer_cast<BehaviorModel>(model_ptr);
}

}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_DYNAMIC_MODEL_HPP_
