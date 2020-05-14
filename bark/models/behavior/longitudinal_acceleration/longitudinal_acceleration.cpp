// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <algorithm>

#include "bark/models/behavior/longitudinal_acceleration/longitudinal_acceleration.hpp"
#include "bark/world/observed_world.hpp"

namespace modules {
namespace models {

dynamic::Trajectory behavior::BehaviorLongitudinalAcceleration::Plan(
    float delta_time, const world::ObservedWorld& observed_world) {
  using namespace dynamic;
  SetBehaviorStatus(BehaviorStatus::VALID);

  //! TODO(@fortiss): parameters
  const float min_velocity = GetMinVelocity();
  const float max_velocity = GetMaxVelocity();
  const int num_traj_time_points = 11;
  dynamic::Trajectory traj(num_traj_time_points,
                           int(StateDefinition::MIN_STATE_SIZE));
  float const dt = delta_time / (num_traj_time_points - 1);

  dynamic::State ego_vehicle_state = observed_world.CurrentEgoState();

  // select state and get p0
  geometry::Point2d pose = observed_world.CurrentEgoPosition();

  auto lane_corr = observed_world.GetLaneCorridor();
  if (!lane_corr) {
    this->SetLastTrajectory(traj);
    return traj;
  }
  geometry::Line line = lane_corr->GetCenterLine();

  // check whether linestring is empty
  if (line.obj_.size() > 0) {
    // adding state at t=0
    traj.block<1, StateDefinition::MIN_STATE_SIZE>(0, 0) =
        ego_vehicle_state.transpose().block<1, StateDefinition::MIN_STATE_SIZE>(
            0, 0);

    float s_start = GetNearestS(line, pose);  // checked
    double start_time = observed_world.GetWorldTime();
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

      geometry::Point2d traj_point = GetPointAtS(line, s_i);  // checked
      float traj_angle = GetTangentAngleAtS(line, s_i);      // checked

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

    SetLastAction(Action(Continuous1DAction(acc)));
  }

  this->SetLastTrajectory(traj);
  return traj;
}

}  // namespace models
}  // namespace modules
