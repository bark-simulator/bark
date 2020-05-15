// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_MOTION_PRIMITIVES_PRIMITIVES_PRIMITIVE_CONST_ACCELERATION_HPP_
#define MODULES_MODELS_BEHAVIOR_MOTION_PRIMITIVES_PRIMITIVES_PRIMITIVE_CONST_ACCELERATION_HPP_

#include "modules/models/behavior/motion_primitives/primitives/primitive.hpp"

namespace modules {
namespace models {
namespace behavior {
namespace primitives {

class PrimitiveConstAcceleration : public Primitive, BehaviorIDMLaneTracking {
  // Covers Primitives KeepVelocity, Accelerat, Decelerate
 public:
  PrimitiveConstAcceleration(const commons::ParamsPtr& params,
                             float acceleration);
  explicit PrimitiveConstAcceleration(const commons::ParamsPtr& params);
  bool IsPreConditionSatisfied(
      const ObservedWorld& observed_world,
      const AdjacentLaneCorridors& adjacent_corridors) override;

  Trajectory Plan(float delta_time, const ObservedWorld& observed_world,
                  const LaneCorridorPtr& target_corridor);

  LaneCorridorPtr SelectTargetCorridor(
      const ObservedWorld& observed_world,
      const AdjacentLaneCorridors& adjacent_corridors) override;

 protected:
  std::pair<double, double> GetTotalAcc(const ObservedWorld& observed_world,
                                        const IDMRelativeValues& rel_values,
                                        double rel_distance,
                                        double dt) const override;

 private:
  float acceleration_;
};

}  // namespace primitives
}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_MOTION_PRIMITIVES_PRIMITIVES_PRIMITIVE_CONST_ACCELERATION_HPP_
