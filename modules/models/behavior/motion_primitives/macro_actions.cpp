// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/models/behavior/motion_primitives/macro_actions.hpp"
#include "modules/models/dynamic/integration.hpp"
#include "modules/world/observed_world.hpp"

namespace modules {
namespace models {
namespace behavior {

using modules::models::dynamic::StateDefinition;

BehaviorMotionPrimitives::MotionIdx
BehaviorMPMacroActions::AddMotionPrimitive(const primitives::PrimitivePtr& primitive) {
  motion_primitives_.push_back(primitive);
  return motion_primitives_.size() - 1;
}

Trajectory BehaviorMPMacroActions::Plan(
    float delta_time, const world::ObservedWorld& observed_world) {
  // TODO: move this to Predicate
  const float dt = integration_time_delta_;
  const int num_trajectory_points =
      static_cast<int>(std::ceil(delta_time / dt)) + 1;

  Trajectory traj(num_trajectory_points,
                  static_cast<int>(StateDefinition::MIN_STATE_SIZE));

  traj = motion_primitives_.at(active_motion_)->Plan(delta_time, observed_world);
  // traj = 

  // SetLastAction(Action(DiscreteAction(active_motion_)));

  this->SetLastTrajectory(traj);
  return traj;
}

}  // namespace behavior
}  // namespace models
}  // namespace modules
