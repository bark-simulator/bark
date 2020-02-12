// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_MOTION_PRIMITIVES_PRIMITIVES_PRIMITIVE_HPP_
#define MODULES_MODELS_BEHAVIOR_MOTION_PRIMITIVES_PRIMITIVES_PRIMITIVE_HPP_

#include "modules/commons/base_type.hpp"
#include "modules/models/behavior/constant_velocity/constant_velocity.hpp"

namespace modules {
namespace world {
class ObservedWorld;
}  // namespace world
namespace models {
namespace behavior {
namespace primitives {

class Primitive : public modules::commons::BaseType {
 public:
  explicit Primitive(commons::Params* params) : commons::BaseType(params) {}

  virtual ~Primitive() {}

  virtual bool IsPreConditionSatisfied(
      const world::ObservedWorld& observed_world) = 0;
  virtual dynamic::Trajectory Plan(
      float delta_time, const world::ObservedWorld& observed_world) = 0;
};

typedef std::shared_ptr<Primitive> PrimitivePtr;

class PrimitiveConstantVelocity : public Primitive {
 public:
  explicit PrimitiveConstantVelocity(commons::Params* params)
      : Primitive(params), behavior_const_vel_(params) {}
  bool IsPreConditionSatisfied(const world::ObservedWorld& observed_world) {
    return true;
  }
  dynamic::Trajectory Plan(float delta_time,
                           const world::ObservedWorld& observed_world) {
    auto traj = behavior_const_vel_.Plan(delta_time, observed_world);
    return traj;
  }
 private:
  modules::models::behavior::BehaviorConstantVelocity behavior_const_vel_;
};

// class PrimitiveGapKeeping : public Primitive {

// }

}  // namespace primitives
}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_MOTION_PRIMITIVES_PRIMITIVES_PRIMITIVE_HPP_