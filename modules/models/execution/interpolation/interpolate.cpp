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

  /*int32_t gr_index = trajectory.rows()-1;
    int32_t sm_index = 0;

    for (int32_t i=0; i<trajectory.rows(); ++i) {
        if(trajectory(i,StateDefinition::TIME_POSITION) >= new_world_time && trajectory(i,StateDefinition::TIME_POSITION) 
                                                                            < trajectory(gr_index,StateDefinition::TIME_POSITION) ) {
            gr_index = i;
        }
        if(trajectory(i,StateDefinition::TIME_POSITION) <= new_world_time ) {
            sm_index = i;
        }
    }

    if(gr_index == sm_index) {
        return StateInputPair(State(trajectory.row(gr_index)),Input::Zero(1,1));
    }
    
    State interp_state = trajectory.row(sm_index) + (trajectory.row(gr_index) - trajectory.row(sm_index))*(new_world_time - trajectory(sm_index,StateDefinition::TIME_POSITION));
    StateInputPair state_input(State(interp_state),Input::Zero(1,1));
    */

  // TODO(fortiss) set trajectory
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> traj = trajectory;
  set_last_trajectory(traj);
  return traj;
}

}  // namespace execution
}  // namespace models
}  // namespace modules
