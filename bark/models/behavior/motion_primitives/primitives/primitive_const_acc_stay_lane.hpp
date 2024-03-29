// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_MODELS_BEHAVIOR_MOTION_PRIMITIVES_PRIMITIVES_PRIMITIVE_CONST_ACC_STAY_LANE_HPP_
#define BARK_MODELS_BEHAVIOR_MOTION_PRIMITIVES_PRIMITIVES_PRIMITIVE_CONST_ACC_STAY_LANE_HPP_

#include "bark/models/behavior/motion_primitives/primitives/primitive.hpp"

namespace bark {
namespace models {
namespace behavior {
namespace primitives {

class PrimitiveConstAccStayLane : public Primitive,
                                  public BehaviorIDMLaneTracking {
  // Covers Primitives KeepVelocity, Accelerat, Decelerate
 public:
  PrimitiveConstAccStayLane(const commons::ParamsPtr& params,
                            double acceleration);
  explicit PrimitiveConstAccStayLane(const commons::ParamsPtr& params);
  bool IsPreConditionSatisfied(
      const ObservedWorld& observed_world,
      const AdjacentLaneCorridors& adjacent_corridors) override;

  Trajectory Plan(double min_planning_time, const ObservedWorld& observed_world,
                  const LaneCorridorPtr& target_corridor);

  std::string GetName() const override;

  double GetAcceleration() const { return acceleration_; }

  IDMRelativeValues CalcRelativeValues(
    const world::ObservedWorld& observed_world,
    const LaneCorridorPtr& lane_corr) const;

  LaneCorridorPtr SelectTargetCorridor(
      const ObservedWorld& observed_world,
      const AdjacentLaneCorridors& adjacent_corridors) override;

 protected:
  std::pair<double, double> GetTotalAcc(const ObservedWorld& observed_world,
                                        const IDMRelativeValues& rel_values,
                                        double rel_distance,
                                        double dt) const override;
  double acceleration_;
  bool restrict_brake_for_lane_end_;

 private:
  bool isEqual(const Primitive& other) const override {
    const PrimitiveConstAccStayLane& other_prim_acc_stay = static_cast<const PrimitiveConstAccStayLane&>(other);
    return acceleration_ == other_prim_acc_stay.acceleration_ && 
      static_cast<const BehaviorIDMLaneTracking&>(*this) == static_cast<const BehaviorIDMLaneTracking&>(other_prim_acc_stay);
  }
};

}  // namespace primitives
}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // BARK_MODELS_BEHAVIOR_MOTION_PRIMITIVES_PRIMITIVES_PRIMITIVE_CONST_ACC_STAY_LANE_HPP_
