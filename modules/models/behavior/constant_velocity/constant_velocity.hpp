// Copyright (c) 2020 Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_MODELS_BEHAVIOR_CONSTANT_VELOCITY_CONSTANT_VELOCITY_HPP_
#define MODULES_MODELS_BEHAVIOR_CONSTANT_VELOCITY_CONSTANT_VELOCITY_HPP_

#include <utility>
#include <memory>

#include "modules/models/behavior/behavior_model.hpp"
#include "modules/models/behavior/idm/idm_classic.hpp"
#include "modules/world/world.hpp"

namespace modules {
namespace models {
namespace behavior {

using dynamic::Trajectory;
using world::objects::AgentId;
using world::ObservedWorld;

// behavior model that drives with a const. vel.
class BehaviorConstantVelocity : public BehaviorIDMClassic {
 public:
  explicit BehaviorConstantVelocity(const commons::ParamsPtr& params) :
    BehaviorIDMClassic(params) {}

  virtual ~BehaviorConstantVelocity() {}

  Trajectory Plan(float min_planning_time,
                  const ObservedWorld& observed_world);

  std::pair<double, double> GetTotalAcc(
    const world::ObservedWorld& observed_world,
    const IDMRelativeValues& rel_values,
    double rel_distance,
    double dt) const;

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
