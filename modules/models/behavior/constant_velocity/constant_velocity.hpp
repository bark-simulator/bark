// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_MODELS_BEHAVIOR_CONSTANT_VELOCITY_CONSTANT_VELOCITY_HPP_
#define MODULES_MODELS_BEHAVIOR_CONSTANT_VELOCITY_CONSTANT_VELOCITY_HPP_

#include "modules/models/behavior/behavior_model.hpp"
#include "modules/models/behavior/idm/idm_classic.hpp"
#include "modules/world/world.hpp"

namespace modules {
namespace models {
namespace behavior {

using dynamic::Trajectory;
using world::objects::AgentId;
using world::ObservedWorld;

class BehaviorConstantVelocity : public BehaviorIDMClassic {
 public:
  explicit BehaviorConstantVelocity(const commons::ParamsPtr& params) :
    BehaviorIDMClassic(params) {}

  virtual ~BehaviorConstantVelocity() {}

  Trajectory Plan(float delta_time,
                  const ObservedWorld& observed_world);

  virtual std::shared_ptr<BehaviorModel> Clone() const;
};

inline std::shared_ptr<BehaviorModel> BehaviorConstantVelocity::Clone() const {
  std::shared_ptr<BehaviorConstantVelocity> model_ptr =
    std::make_shared<BehaviorConstantVelocity>(*this);
  return model_ptr;
}

}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_CONSTANT_VELOCITY_CONSTANT_VELOCITY_HPP_
