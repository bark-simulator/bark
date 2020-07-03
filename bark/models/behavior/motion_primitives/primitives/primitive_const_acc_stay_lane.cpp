// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "primitive_const_acc_stay_lane.hpp"

bark::models::behavior::primitives::PrimitiveConstAccStayLane::
    PrimitiveConstAccStayLane(const bark::commons::ParamsPtr& params)
    : Primitive(params),
      BehaviorModel(params),
      BehaviorIDMLaneTracking(params),
      acceleration_(params->GetReal("PrimitiveConstAccStayLane::Acceleration",
                                    "Constant acceleration to apply", 0.0)) {}

bark::models::behavior::primitives::PrimitiveConstAccStayLane::
    PrimitiveConstAccStayLane(const bark::commons::ParamsPtr& params,
                              float acceleration)
    : Primitive(params),
      BehaviorModel(params),
      BehaviorIDMLaneTracking(params),
      acceleration_(acceleration) {}

bool bark::models::behavior::primitives::PrimitiveConstAccStayLane::
    IsPreConditionSatisfied(
        const bark::world::ObservedWorld& observed_world,
        const bark::models::behavior::primitives::AdjacentLaneCorridors&
            adjacent_corridors) {
  auto single_track = std::dynamic_pointer_cast<dynamic::SingleTrackModel>(
      observed_world.GetEgoAgent()->GetDynamicModel());
  if (!single_track) {
    LOG(FATAL) << "Only single track model supported! Aborting!";
  }
  auto ego_state = observed_world.CurrentEgoState();
  return acceleration_ >= single_track->GetMinAcceleration(ego_state) &&
         acceleration_ <= single_track->GetMaxAcceleration(ego_state);
}

bark::models::dynamic::Trajectory
bark::models::behavior::primitives::PrimitiveConstAccStayLane::Plan(
    float delta_time, const bark::world::ObservedWorld& observed_world,
    const bark::world::LaneCorridorPtr& target_corridor) {
  SetBehaviorStatus(BehaviorStatus::VALID);

  if (!target_corridor) {
    LOG(INFO) << "Agent " << observed_world.GetEgoAgentId()
              << ": Behavior status has expired!" << std::endl;
    SetBehaviorStatus(BehaviorStatus::EXPIRED);
    return GetLastTrajectory();
  }

  double dt = delta_time / (GetNumTrajectoryTimePoints() - 1);
  // interaction term off and GetTotalAcc returns const. acc.
  IDMRelativeValues rel_values{0., 0., false};
  std::tuple<Trajectory, Action> traj_action =
      GenerateTrajectory(observed_world, target_corridor, rel_values, dt);

  // set values
  Trajectory traj = std::get<0>(traj_action);
  Action action = std::get<1>(traj_action);
  SetLastTrajectory(traj);
  Primitive::SetLastAction(action);
  BehaviorModel::SetLastAction(action);
  return traj;
}

bark::world::LaneCorridorPtr bark::models::behavior::primitives::
    PrimitiveConstAccStayLane::SelectTargetCorridor(
        const bark::world::ObservedWorld& observed_world,
        const bark::models::behavior::primitives::AdjacentLaneCorridors&
            adjacent_corridors) {
  BARK_EXPECT_TRUE(adjacent_corridors.current);
  return adjacent_corridors.current;
}

std::pair<double, double>
bark::models::behavior::primitives::PrimitiveConstAccStayLane::GetTotalAcc(
    const bark::world::ObservedWorld& observed_world,
    const bark::models::behavior::IDMRelativeValues& rel_values,
    double rel_distance, double dt) const {
  return {acceleration_, 0.0f};
}
