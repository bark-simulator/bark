// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/models/behavior/motion_primitives/macro_actions.hpp"
#include "bark/models/dynamic/integration.hpp"
#include "bark/world/observed_world.hpp"

namespace bark {
namespace models {
namespace behavior {

using bark::models::dynamic::StateDefinition;

BehaviorMPMacroActions::BehaviorMPMacroActions(
    const commons::ParamsPtr& params,
    const std::vector<primitives::PrimitivePtr>& motion_primitives)
    : BehaviorMotionPrimitives(params),
      motion_primitives_(motion_primitives),
      check_validity_in_plan_(params->GetBool(
          "BehaviorMPMacroActions::CheckValidityInPlan",
          "If true only primitives can be selected which are valid", true)) {}

BehaviorMotionPrimitives::MotionIdx BehaviorMPMacroActions::AddMotionPrimitive(
    const primitives::PrimitivePtr& primitive) {
  motion_primitives_.push_back(primitive);
  return motion_primitives_.size() - 1;
}

Trajectory BehaviorMPMacroActions::Plan(
    float delta_time, const world::ObservedWorld& observed_world) {
  SetBehaviorStatus(BehaviorStatus::VALID);
  const float dt = integration_time_delta_;
  const int num_trajectory_points =
      static_cast<int>(std::ceil(delta_time / dt)) + 1;

  Trajectory traj(num_trajectory_points,
                  static_cast<int>(StateDefinition::MIN_STATE_SIZE));

  primitives::PrimitivePtr selected_mp;
  AdjacentLaneCorridors adjacent_corridors = GetCorridors(observed_world);
  if (check_validity_in_plan_) {
    // SetLastAction(Action(DiscreteAction(active_motion_)));

    // There must be at least one primitive that is always available!
    if (valid_primitives_.empty()) {
      GetNumMotionPrimitivesByCorridors(observed_world, adjacent_corridors);
      LOG_IF(ERROR, valid_primitives_.empty())
          << "No motion primitive available! At least one primitive must be "
             "available at all times!";
    }
    selected_mp = motion_primitives_.at(
        valid_primitives_.at(boost::get<DiscreteAction>(active_motion_)));
  } else {
    selected_mp =
        motion_primitives_.at(boost::get<DiscreteAction>(active_motion_));
  }
  target_corridor_ =
      selected_mp->SelectTargetCorridor(observed_world, adjacent_corridors);

  traj = selected_mp->Plan(delta_time, observed_world, target_corridor_);

  this->SetLastAction(
      motion_primitives_.at(boost::get<DiscreteAction>(active_motion_))
          ->GetLastAction());
  this->SetLastTrajectory(traj);
  return traj;
}
const std::vector<primitives::PrimitivePtr>&
BehaviorMPMacroActions::GetMotionPrimitives() const {
  return motion_primitives_;
}
BehaviorMotionPrimitives::MotionIdx
BehaviorMPMacroActions::GetNumMotionPrimitives(
    const ObservedWorldPtr& observed_world) {
  return GetNumMotionPrimitivesByCorridors(*observed_world,
                                           GetCorridors(*observed_world));
}
AdjacentLaneCorridors BehaviorMPMacroActions::GetCorridors(
    const ObservedWorld& observed_world) {
  if (!target_corridor_) {
    target_corridor_ = observed_world.GetLaneCorridor();
  }
  AdjacentLaneCorridors adjacent_corridors;
  const auto& ego_pos = observed_world.CurrentEgoPosition();
  auto point_on_target_line =
      GetNearestPoint(target_corridor_->GetCenterLine(), ego_pos);
  auto road_corridor = observed_world.GetRoadCorridor();
  std::tie(adjacent_corridors.left, adjacent_corridors.right) =
      road_corridor->GetLeftRightLaneCorridor(point_on_target_line);
  adjacent_corridors.current = target_corridor_;
  return adjacent_corridors;
}
BehaviorMotionPrimitives::MotionIdx
BehaviorMPMacroActions::GetNumMotionPrimitivesByCorridors(
    const ObservedWorld& observed_world,
    const AdjacentLaneCorridors& adjacent_corridors) {
  MotionIdx i = 0;
  valid_primitives_.clear();
  for (auto const& p : motion_primitives_) {
    if (p->IsPreConditionSatisfied(observed_world, adjacent_corridors)) {
      valid_primitives_.push_back(i);
    }
    ++i;
  }
  return valid_primitives_.size();
}
const std::vector<BehaviorMPMacroActions::MotionIdx>&
BehaviorMPMacroActions::GetValidPrimitives(
    const ObservedWorldPtr& observed_world) {
  GetNumMotionPrimitives(observed_world);
  return valid_primitives_;
}
inline std::shared_ptr<BehaviorModel> BehaviorMPMacroActions::Clone() const {
  std::shared_ptr<BehaviorMPMacroActions> model_ptr =
      std::make_shared<BehaviorMPMacroActions>(*this);
  return model_ptr;
}

}  // namespace behavior
}  // namespace models
}  // namespace bark
