// Copyright (c) 2020 Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_MOTION_PRIMITIVES_PRIMITIVES_PRIMITIVE_CONST_ACC_STAY_LANE_HPP_
#define MODULES_MODELS_BEHAVIOR_MOTION_PRIMITIVES_PRIMITIVES_PRIMITIVE_CONST_ACC_STAY_LANE_HPP_

#include "modules/models/behavior/motion_primitives/primitives/primitive.hpp"

namespace modules {
namespace models {
namespace behavior {
namespace primitives {

class PrimitiveConstAccStayLane : public Primitive, BehaviorIDMLaneTracking {
  // Covers Primitives KeepVelocity, Accelerat, Decelerate
 public:
  PrimitiveConstAccStayLane(const commons::ParamsPtr& params,
                            float acceleration);
  explicit PrimitiveConstAccStayLane(const commons::ParamsPtr& params);
  bool IsPreConditionSatisfied(
      const ObservedWorld& observed_world,
      const AdjacentLaneCorridors& adjacent_corridors) override;

  Trajectory Plan(float min_dt, const ObservedWorld& observed_world,
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

#endif  // MODULES_MODELS_BEHAVIOR_MOTION_PRIMITIVES_PRIMITIVES_PRIMITIVE_CONST_ACC_STAY_LANE_HPP_
