// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_MODELS_BEHAVIOR_MOTION_PRIMITIVES_PRIMITIVES_PRIMITIVE_CONST_ACC_CHANGE_TO_RIGHT_HPP_
#define BARK_MODELS_BEHAVIOR_MOTION_PRIMITIVES_PRIMITIVES_PRIMITIVE_CONST_ACC_CHANGE_TO_RIGHT_HPP_

#include "bark/models/behavior/motion_primitives/primitives/primitive_const_acc_stay_lane.hpp"

namespace bark {
namespace models {
namespace behavior {
namespace primitives {

/**
 * @brief Change target corridor to the right with constant acceleration
 */
class PrimitiveConstAccChangeToRight : public PrimitiveConstAccStayLane {
 public:
  explicit PrimitiveConstAccChangeToRight(const commons::ParamsPtr& params);

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
}  // namespace bark

#endif  // BARK_MODELS_BEHAVIOR_MOTION_PRIMITIVES_PRIMITIVES_PRIMITIVE_CONST_ACC_CHANGE_TO_RIGHT_HPP_
