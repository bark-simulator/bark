// Copyright (c) 2021 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/models/behavior/behavior_model.hpp"
#include "bark/commons/timer/timer.hpp"

namespace bark {
namespace models {
namespace behavior {

Trajectory BehaviorModel::PlanBehavior(double min_planning_time,
                                   const world::ObservedWorld& observed_world) {
  Trajectory traj;
  if (measure_solution_time_) {
    auto timer = bark::commons::timer::Timer();
    timer.Start();
    traj = this->Plan(min_planning_time, observed_world);
    double duration = timer.DurationInSeconds();
    this->SetLastSolutionTime(duration);
  } else {
    traj = this->Plan(min_planning_time, observed_world);
  }
  return traj;
}
}  // namespace behavior
}  // namespace models
}  // namespace bark
