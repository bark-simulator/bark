// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include <iostream>
#include "modules/models/execution/interpolation/interpolate.hpp"


namespace modules {
namespace models {
namespace execution {

dynamic::Trajectory ExecutionModelInterpolate::Execute(
    const float& new_world_time,
    const dynamic::Trajectory& trajectory,
    const dynamic::DynamicModelPtr dynamic_model,
    const dynamic::State current_state) {

  // TODO(fortiss) fix interpolation model
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> traj = trajectory;
  set_last_trajectory(traj);
  return traj;
}

}  // namespace execution
}  // namespace models
}  // namespace modules
