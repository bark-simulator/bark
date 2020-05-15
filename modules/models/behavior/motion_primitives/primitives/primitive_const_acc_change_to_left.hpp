// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_MOTION_PRIMITIVES_PRIMITIVES_PRIMITIVE_CONST_ACC_CHANGE_TO_LEFT_HPP_
#define MODULES_MODELS_BEHAVIOR_MOTION_PRIMITIVES_PRIMITIVES_PRIMITIVE_CONST_ACC_CHANGE_TO_LEFT_HPP_

#include "modules/models/behavior/motion_primitives/primitives/primitive_const_acceleration.hpp"

namespace modules {
namespace models {
namespace behavior {
namespace primitives {

// TODO(@esterle, @bernhard): Add documentation
class PrimitiveConstAccChangeToLeft : public PrimitiveConstAcceleration {
 public:
  explicit PrimitiveConstAccChangeToLeft(const commons::ParamsPtr& params);

  LaneCorridorPtr SelectTargetCorridor(
      const ObservedWorld& observed_world,
      const AdjacentLaneCorridors& adjacent_corridors) override;

  bool IsPreConditionSatisfied(
      const ObservedWorld& observed_world,
      const AdjacentLaneCorridors& adjacent_corridors) override;

 private:
  float min_length_;
};

}  // namespace primitives
}  // namespace behavior
}  // namespace models
}  // namespace modules


#endif  // MODULES_MODELS_BEHAVIOR_MOTION_PRIMITIVES_PRIMITIVES_PRIMITIVE_CONST_ACC_CHANGE_TO_LEFT_HPP_
