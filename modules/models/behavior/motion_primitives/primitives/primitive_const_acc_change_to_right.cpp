// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "primitive_const_acc_change_to_right.hpp"
modules::models::behavior::primitives::PrimitiveConstAccChangeToRight::
    PrimitiveConstAccChangeToRight(const modules::commons::ParamsPtr& params)
    : PrimitiveConstAccStayLane(params),
      min_length_(params->GetReal(
          "MinLength", "Minimum length of lane to change to", 0.0f)) {}
modules::world::LaneCorridorPtr modules::models::behavior::primitives::
    PrimitiveConstAccChangeToRight::SelectTargetCorridor(
        const modules::world::ObservedWorld& observed_world,
        const modules::models::behavior::primitives::AdjacentLaneCorridors&
            adjacent_corridors) {
  if (adjacent_corridors.right) {
    return adjacent_corridors.right;
  }
  LOG(WARNING) << "Called change to right, but right corridor not found!";
  if (!adjacent_corridors.current) {
    return observed_world.GetRoadCorridor()->GetCurrentLaneCorridor(
        observed_world.CurrentEgoPosition());
  } else {
    return adjacent_corridors.current;
  }
}
bool modules::models::behavior::primitives::PrimitiveConstAccChangeToRight::
    IsPreConditionSatisfied(
        const modules::world::ObservedWorld& observed_world,
        const modules::models::behavior::primitives::AdjacentLaneCorridors&
            adjacent_corridors) {
  bool satisfied;
  if (adjacent_corridors.right) {
    const Point2d ego_pos = observed_world.CurrentEgoPosition();
    //! agent may not have reached target lane yet, so we match point on target
    //! lane
    const Point2d point_on_target_line =
        GetNearestPoint(adjacent_corridors.right->GetCenterLine(), ego_pos);
    float remaining_length =
        adjacent_corridors.right->LengthUntilEnd(point_on_target_line);
    satisfied = remaining_length >= min_length_;
  } else {
    satisfied = false;
  }
  return satisfied;
}
