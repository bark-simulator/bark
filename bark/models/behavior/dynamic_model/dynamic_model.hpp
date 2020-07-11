// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_MODELS_BEHAVIOR_DYNAMIC_MODEL_HPP_
#define BARK_MODELS_BEHAVIOR_DYNAMIC_MODEL_HPP_

#include "bark/models/behavior/behavior_model.hpp"
#include "bark/models/dynamic/dynamic_model.hpp"
#include "bark/world/world.hpp"

namespace bark {
namespace models {
namespace behavior {

using dynamic::DynamicModelPtr;
using dynamic::Input;
using dynamic::State;
using dynamic::Trajectory;
using world::ObservedWorld;
using world::objects::AgentId;

// model that uses last_action_ to produce trajectory
// can e.g. be used for RL
class BehaviorDynamicModel : public BehaviorModel {
 public:
  explicit BehaviorDynamicModel(const commons::ParamsPtr& params);

  virtual ~BehaviorDynamicModel() {}

  virtual Trajectory Plan(float delta_time,
                          const ObservedWorld& observed_world);

  virtual std::shared_ptr<BehaviorModel> Clone() const;

  virtual void ActionToBehavior(const Action& action) { action_ = action; }

 private:
  float integration_time_delta_;
  Action action_;
};

inline std::shared_ptr<BehaviorModel> BehaviorDynamicModel::Clone() const {
  std::shared_ptr<BehaviorDynamicModel> model_ptr =
      std::make_shared<BehaviorDynamicModel>(*this);
  return model_ptr;
}

}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // BARK_MODELS_BEHAVIOR_DYNAMIC_MODEL_HPP_
