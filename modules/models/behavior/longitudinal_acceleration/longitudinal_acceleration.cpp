// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <algorithm>

#include "modules/models/behavior/longitudinal_acceleration/longitudinal_acceleration.hpp"
#include "modules/world/observed_world.hpp"

namespace modules {
namespace models {

dynamic::Trajectory behavior::BehaviorLongitudinalAcceleration::Plan(
    float delta_time, const world::ObservedWorld& observed_world) {
  using namespace dynamic;

  //! TODO(@fortiss): parameters
  const float min_velocity = get_min_velocity();
  const float max_velocity = get_max_velocity();
  const int num_traj_time_points = 11;
  dynamic::Trajectory traj(num_traj_time_points,
                           int(StateDefinition::MIN_STATE_SIZE));
  float const dt = delta_time / (num_traj_time_points - 1);

  dynamic::State ego_vehicle_state = observed_world.current_ego_state();

  // select state and get p0
  geometry::Point2d pose(ego_vehicle_state(StateDefinition::X_POSITION),
                         ego_vehicle_state(StateDefinition::Y_POSITION));

  geometry::Line line = observed_world.get_local_map()
                            ->get_driving_corridor()
                            .get_center();  // checked

  // check whether linestring is empty
  if (line.obj_.size() > 0) {
    // adding state at t=0
    traj.block<1, StateDefinition::MIN_STATE_SIZE>(0, 0) =
        ego_vehicle_state.transpose().block<1, StateDefinition::MIN_STATE_SIZE>(
            0, 0);

    float s_start = get_nearest_s(line, pose);  // checked
    double start_time = observed_world.get_world_time();
    float vel_i = ego_vehicle_state(StateDefinition::VEL_POSITION);
    double acc = CalculateLongitudinalAcceleration(observed_world);  // checked
    BARK_EXPECT_TRUE(!std::isnan(acc));
    float s_i = s_start;
    double t_i;

    for (int i = 1; i < num_traj_time_points; ++i) {
      s_i += +0.5f * acc * dt * dt + vel_i * dt;
      const float temp_velocity = vel_i + acc * dt;
      vel_i = std::max(std::min(temp_velocity, max_velocity), min_velocity);
      t_i = static_cast<float>(i) * dt + start_time;

      geometry::Point2d traj_point = get_point_at_s(line, s_i);  // checked
      float traj_angle = get_tangent_angle_at_s(line, s_i);      // checked

      BARK_EXPECT_TRUE(!std::isnan(boost::geometry::get<0>(traj_point)));
      BARK_EXPECT_TRUE(!std::isnan(boost::geometry::get<1>(traj_point)));
      BARK_EXPECT_TRUE(!std::isnan(traj_angle));

      traj(i, StateDefinition::TIME_POSITION) = t_i;  // checked
      traj(i, StateDefinition::X_POSITION) =
          boost::geometry::get<0>(traj_point);  // checked
      traj(i, StateDefinition::Y_POSITION) =
          boost::geometry::get<1>(traj_point);                // checked
      traj(i, StateDefinition::THETA_POSITION) = traj_angle;  // checked
      traj(i, StateDefinition::VEL_POSITION) = vel_i;         // checked
    }

    set_last_action(Action(Continuous1DAction(acc)));
  }

  this->set_last_trajectory(traj);
  return traj;
}

}  // namespace models
}  // namespace modules
