// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_MODELS_BEHAVIOR_CONSTANT_VELOCITY_CONSTANT_ACCELERATION_HPP_
#define BARK_MODELS_BEHAVIOR_CONSTANT_VELOCITY_CONSTANT_ACCELERATION_HPP_

#include <memory>
#include <utility>

#include "bark/models/behavior/behavior_model.hpp"
#include "bark/models/behavior/idm/idm_classic.hpp"
#include "bark/world/world.hpp"

namespace bark {
namespace models {
namespace behavior {

using dynamic::Trajectory;
using world::ObservedWorld;
using world::objects::AgentId;

// behavior model that drives with a const. vel.
class BehaviorConstantAcceleration : public BehaviorIDMClassic {
 public:
  explicit BehaviorConstantAcceleration(const commons::ParamsPtr& params)
      : BehaviorModel(params), BehaviorIDMClassic(params) {
    const_acc_ =
      params->GetReal(
        "BehaviorConstantAcceleration::ConstAcceleration",
        "Constant acceleration for vehicle.",
        0.f);
  }

  virtual ~BehaviorConstantAcceleration() {}

  Trajectory Plan(float min_planning_time, const ObservedWorld& observed_world);

  std::pair<double, double> GetTotalAcc(
      const world::ObservedWorld& observed_world,
      const IDMRelativeValues& rel_values, double rel_distance,
      double dt) const;

  virtual std::shared_ptr<BehaviorModel> Clone() const;

 private:
  double const_acc_;
};

inline std::shared_ptr<BehaviorModel> BehaviorConstantAcceleration::Clone() const {
  std::shared_ptr<BehaviorConstantAcceleration> model_ptr =
      std::make_shared<BehaviorConstantAcceleration>(*this);
  return model_ptr;
}

}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // BARK_MODELS_BEHAVIOR_CONSTANT_VELOCITY_CONSTANT_ACCELERATION_HPP_
