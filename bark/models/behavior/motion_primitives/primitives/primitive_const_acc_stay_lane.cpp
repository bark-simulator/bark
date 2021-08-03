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
                                    "Constant acceleration to apply", 0.0)),
      restrict_brake_for_lane_end_(params->GetBool("PrimitiveConstAccStayLane::RestrictBrakeForLaneEnd",
                                    "Acceleration restricted by brake for lane end", false)) {}

bark::models::behavior::primitives::PrimitiveConstAccStayLane::
    PrimitiveConstAccStayLane(const bark::commons::ParamsPtr& params,
                              double acceleration)
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
  return acceleration_ >= single_track->GetLonAccelerationMin(ego_state) &&
         acceleration_ <= single_track->GetLonAccelerationMax(ego_state);
}

bark::models::dynamic::Trajectory
bark::models::behavior::primitives::PrimitiveConstAccStayLane::Plan(
    double delta_time, const bark::world::ObservedWorld& observed_world,
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
  if(restrict_brake_for_lane_end_) {
    const auto current_corridor = observed_world.GetRoadCorridor()->GetCurrentLaneCorridor(
                                    observed_world.CurrentEgoPosition());
    // interaction term on only for brake for lane end in current corridor
    rel_values = CalcRelativeValues(observed_world, target_corridor);
  } 

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

bark::models::behavior::IDMRelativeValues bark::models::behavior::primitives::PrimitiveConstAccStayLane::CalcRelativeValues(
    const world::ObservedWorld& observed_world,
    const LaneCorridorPtr& lane_corr) const {
  bool interaction_term_active = false;
  double leading_distance = 0.;
  double leading_velocity = 1e6;
  double ego_acc = 0.0;  // TODO: ommit, as it is confusing
  double leading_acc = 0.0;
  IDMRelativeValues rel_values;
  // 2nd part for lane_corr end
  if (brake_lane_end_) {
    bool braking_required;
    double len_until_end;
    std::tie(braking_required, len_until_end) =
        GetDistanceToLaneEnding(lane_corr, observed_world.CurrentEgoPosition());
    if (braking_required) {
      interaction_term_active = true;
        leading_distance = len_until_end;
        leading_velocity = 0.;
    }
  }

  Action ego_action = BaseIDM::GetLastAction();
  if (ego_action.type() == typeid(Continuous1DAction)) {
    ego_acc = boost::get<Continuous1DAction>(ego_action);
  } else if (ego_action.type() == typeid(LonLatAction)) {
    ego_acc = boost::get<LonLatAction>(ego_action).acc_lon;
  } else if (ego_action.type() == typeid(Input)) {
    ego_acc = boost::get<Input>(ego_action)(0);
  } else {
    LOG(FATAL) << "ego action type unknown: "
               << boost::apply_visitor(action_tostring_visitor(), ego_action);
  }

  rel_values.leading_distance = leading_distance;
  rel_values.leading_velocity = leading_velocity;
  rel_values.ego_acc = ego_acc;
  rel_values.leading_acc = leading_acc;
  rel_values.has_leading_object = interaction_term_active;
  return rel_values;
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
  double acc = acceleration_;
  if(restrict_brake_for_lane_end_) {
    double idm_acc;
    std::tie(idm_acc, rel_distance) =
          BaseIDM::GetTotalAcc(observed_world, rel_values, rel_distance, dt);
    acc = std::min(idm_acc, acceleration_);
  }
  return {acc, rel_distance};
}

std::string bark::models::behavior::primitives::PrimitiveConstAccStayLane::GetName() const {
  std::stringstream ss;
  ss << "PrimitiveConstAccStayLane: a=" << acceleration_;
  return ss.str();
 }
