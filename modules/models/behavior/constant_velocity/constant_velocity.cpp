// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include "modules/models/behavior/constant_velocity/constant_velocity.hpp"
#include "modules/world/observed_world.hpp"

namespace modules {
namespace models {

dynamic::Trajectory behavior::BehaviorConstantVelocity::Plan(
    float delta_time,
    const world::ObservedWorld& observed_world) {

  using namespace dynamic;

  //! TODO(@fortiss): parameters
  const int num_traj_time_points = 100;
  dynamic::Trajectory traj(num_traj_time_points, int(StateDefinition::MIN_STATE_SIZE));
  auto const sample_time = delta_time / num_traj_time_points;

  dynamic::State ego_vehicle_state = observed_world.get_ego_state();
  
  // select state and get p0
  geometry::Point2d pose(ego_vehicle_state(StateDefinition::X_POSITION),
                          ego_vehicle_state(StateDefinition::Y_POSITION));

  
  geometry::Line line = observed_world.get_local_map()->get_center_line();
  // check whether linestring is empty
  if (line.obj_.size()>0) {
    float s_start = get_nearest_s(line, pose);
    double start_time = observed_world.get_world_time();
    float constant_vel = ego_vehicle_state(StateDefinition::VEL_POSITION);
    // v = s/t
    double run_time = start_time;
    for (int i = 0; i < traj.rows(); i++) {
      float del_s = constant_vel * (run_time - start_time);
      geometry::Point2d traj_point = get_point_at_s(line, s_start + del_s);
      float traj_angle = get_tangent_angle_at_s(line, s_start + del_s);
      traj(i, StateDefinition::TIME_POSITION) = run_time;
      traj(i, StateDefinition::X_POSITION) = boost::geometry::get<0>(traj_point);
      traj(i, StateDefinition::Y_POSITION) = boost::geometry::get<1>(traj_point);
      traj(i, StateDefinition::THETA_POSITION) = traj_angle;
      traj(i, StateDefinition::VEL_POSITION) = constant_vel;
      run_time += sample_time;
    }
  }
  this->set_last_trajectory(traj);
  return traj;
}

}  // namespace models
}  // namespace modules
