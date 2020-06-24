// Copyright (c) 2020 Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_MODELS_BEHAVIOR_MOTION_PRIMITIVES_PRIMITIVES_PRIMITIVE_GAP_KEEPING_HPP_
#define BARK_MODELS_BEHAVIOR_MOTION_PRIMITIVES_PRIMITIVES_PRIMITIVE_GAP_KEEPING_HPP_

#include "bark/models/behavior/motion_primitives/primitives/primitive.hpp"

namespace bark {
namespace models {
namespace behavior {
namespace primitives {

/**
 * @brief Lane tracking with IDM for longitudinal motion
 */
class PrimitiveGapKeeping : public Primitive, BehaviorIDMLaneTracking {
 public:
  explicit PrimitiveGapKeeping(const commons::ParamsPtr& params)
      : Primitive(params), BehaviorIDMLaneTracking(params) {}
  bool IsPreConditionSatisfied(
      const ObservedWorld& observed_world,
      const AdjacentLaneCorridors& adjacent_corridors) override {
    return true;
  }
  Trajectory Plan(float min_planning_time, const ObservedWorld& observed_world,
                  const LaneCorridorPtr& target_corridor) override {
    return BehaviorIDMLaneTracking::Plan(min_planning_time, observed_world);
  }
  LaneCorridorPtr SelectTargetCorridor(
      const ObservedWorld& observed_world,
      const AdjacentLaneCorridors& adjacent_corridors) override {
    return observed_world.GetRoadCorridor()->GetCurrentLaneCorridor(
        observed_world.CurrentEgoPosition());
  }
};

}  // namespace primitives
}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // BARK_MODELS_BEHAVIOR_MOTION_PRIMITIVES_PRIMITIVES_PRIMITIVE_GAP_KEEPING_HPP_
