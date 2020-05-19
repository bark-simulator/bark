// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "primitive_const_acc_change_to_left.hpp"
modules::models::behavior::primitives::PrimitiveConstAccChangeToLeft::
    PrimitiveConstAccChangeToLeft(const modules::commons::ParamsPtr& params)
    : PrimitiveConstAccStayLane(params),
      min_length_(params->GetReal(
          "MinLength", "Minimum length of lane to change to", 0.0f)) {}
modules::world::LaneCorridorPtr modules::models::behavior::primitives::
    PrimitiveConstAccChangeToLeft::SelectTargetCorridor(
        const modules::world::ObservedWorld& observed_world,
        const modules::models::behavior::primitives::AdjacentLaneCorridors&
            adjacent_corridors) {
  if (adjacent_corridors.left) {
    return adjacent_corridors.left;
  }
  LOG(WARNING) << "Called change to left, but left corridor not found!";
  if (!adjacent_corridors.current) {
    return observed_world.GetRoadCorridor()->GetCurrentLaneCorridor(
        observed_world.CurrentEgoPosition());
  } else {
    return adjacent_corridors.current;
  }
}
bool modules::models::behavior::primitives::PrimitiveConstAccChangeToLeft::
    IsPreConditionSatisfied(
        const modules::world::ObservedWorld& observed_world,
        const modules::models::behavior::primitives::AdjacentLaneCorridors&
            adjacent_corridors) {
  bool satisfied = false;
  if (adjacent_corridors.left) {
    const Point2d ego_pos = observed_world.CurrentEgoPosition();
    //! agent may not have reached target lane yet, so we match point on target
    //! lane
    const Point2d point_on_target_line =
        GetNearestPoint(adjacent_corridors.left->GetCenterLine(), ego_pos);
    float remaining_length =
        adjacent_corridors.left->LengthUntilEnd(point_on_target_line);
    satisfied = remaining_length >= min_length_;
  }
  return satisfied;
}
