// Copyright (c) 2020 Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include <iostream>
#include "modules/models/execution/interpolation/interpolate.hpp"


namespace modules {
namespace models {
namespace execution {



State ExecutionModelInterpolate::Execute(
  const float& new_world_time,
  const dynamic::Trajectory& trajectory,
  const dynamic::DynamicModelPtr dynamic_model) {

  
  // TODO(@hart): fix interpolation model
  int index_world_time = 0;
  float min_time_diff = std::numeric_limits<float>::max();
  for (int i = 0; i < trajectory.rows(); i++) {
    float diff_time = fabs(trajectory(i, dynamic::TIME_POSITION) - new_world_time);
    if (diff_time < min_time_diff) {
      index_world_time = i;
      min_time_diff = diff_time;
    }
  }

  SetLastTrajectory(trajectory);
  SetLastState(State(trajectory.row(index_world_time)));
  return State(trajectory.row(index_world_time));
}

}  // namespace execution
}  // namespace models
}  // namespace modules
