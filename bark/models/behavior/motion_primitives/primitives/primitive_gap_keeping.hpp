// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
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
class PrimitiveGapKeeping : public Primitive, public BehaviorIDMLaneTracking {
 public:
  explicit PrimitiveGapKeeping(const commons::ParamsPtr& params)
      : Primitive(params),
        BehaviorModel(params),
        BehaviorIDMLaneTracking(params) {
    Primitive::SetLastAction(Continuous1DAction(0.0f));
  }
  bool IsPreConditionSatisfied(
      const ObservedWorld& observed_world,
      const AdjacentLaneCorridors& adjacent_corridors) override {
    return true;
  }
  Trajectory Plan(float min_planning_time, const ObservedWorld& observed_world,
                  const LaneCorridorPtr& target_corridor) override {
    auto traj =
        BehaviorIDMLaneTracking::Plan(min_planning_time, observed_world);
    Primitive::SetLastAction(BehaviorIDMLaneTracking::GetLastAction());
    return traj;
  }

  Action GetLastAction() const {
    return BehaviorIDMLaneTracking::GetLastAction();
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
