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
    Primitive::SetLastAction(Continuous1DAction(0.0));
  }
  bool IsPreConditionSatisfied(
      const ObservedWorld& observed_world,
      const AdjacentLaneCorridors& adjacent_corridors) override {
    return true;
  }
  Trajectory Plan(double min_planning_time, const ObservedWorld& observed_world,
                  const LaneCorridorPtr& target_corridor) override {
    auto traj =
        BehaviorIDMLaneTracking::Plan(min_planning_time, observed_world);
    Primitive::SetLastAction(BehaviorIDMLaneTracking::GetLastAction());
    return traj;
  }

  std::string GetName() const override { return "PrimitiveGapKeeping"; }

  Action GetLastAction() const {
    return BehaviorIDMLaneTracking::GetLastAction();
  }

  LaneCorridorPtr SelectTargetCorridor(
      const ObservedWorld& observed_world,
      const AdjacentLaneCorridors& adjacent_corridors) override {
    return observed_world.GetRoadCorridor()->GetCurrentLaneCorridor(
        observed_world.CurrentEgoPosition());
  }
private:
  bool isEqual(const Primitive& other) const override {
    const PrimitiveGapKeeping& other_prim_gap_keep = static_cast<const PrimitiveGapKeeping&>(other);
    return static_cast<const BehaviorIDMLaneTracking&>(*this) == static_cast<const BehaviorIDMLaneTracking&>(other_prim_gap_keep);
  }
};

}  // namespace primitives
}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // BARK_MODELS_BEHAVIOR_MOTION_PRIMITIVES_PRIMITIVES_PRIMITIVE_GAP_KEEPING_HPP_
