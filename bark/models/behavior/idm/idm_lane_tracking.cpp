// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/models/behavior/idm/idm_lane_tracking.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <tuple>
#include <utility>

#include "bark/commons/transformation/frenet.hpp"
#include "bark/models/dynamic/integration.hpp"
#include "bark/models/dynamic/single_track.hpp"
#include "bark/world/observed_world.hpp"

namespace bark {
namespace models {
namespace behavior {

using bark::commons::transformation::FrenetPosition;
using bark::geometry::Point2d;
using bark::models::dynamic::CalculateLateralAcceleration;
using bark::models::dynamic::DynamicModelPtr;
using bark::models::dynamic::State;
using bark::models::dynamic::StateDefinition;
using bark::world::objects::Agent;
;
using bark::world::objects::AgentPtr;

std::tuple<Trajectory, Action> BehaviorIDMLaneTracking::GenerateTrajectory(
    const world::ObservedWorld& observed_world,
    const LaneCorridorPtr& lane_corr, const IDMRelativeValues& rel_values,
    double dt) const {
  // definitions
  const DynamicModelPtr dynamic_model =
      observed_world.GetEgoAgent()->GetDynamicModel();
  auto single_track =
      std::dynamic_pointer_cast<dynamic::SingleTrackModel>(dynamic_model);
  if (!single_track) {
    LOG(FATAL) << "Only SingleTrack as dynamic model supported!";
  } else {
    single_track->SetAccelerationLimits(GetAccelerationLimits());
  }

  dynamic::State ego_vehicle_state = observed_world.CurrentEgoState();
  double start_time = observed_world.GetWorldTime();
  double t_i = 0., acc = 0.;
  geometry::Line line;
  if (constant_lane_corr_ != nullptr) {
    // std::cout << "using const. lane corr: " << constant_lane_corr_ <<
    // std::endl;
    line = constant_lane_corr_->GetCenterLine();
  } else {
    line = lane_corr->GetCenterLine();
  }
  dynamic::Trajectory traj(GetNumTrajectoryTimePoints(),
                           static_cast<int>(StateDefinition::MIN_STATE_SIZE));

  double initial_acceleration = 0.0f;
  if (!line.obj_.empty()) {
    // adding state at t=0
    traj.block<1, StateDefinition::MIN_STATE_SIZE>(0, 0) =
        ego_vehicle_state.transpose().block<1, StateDefinition::MIN_STATE_SIZE>(
            0, 0);
    traj(0, StateDefinition::TIME_POSITION) = start_time;

    double rel_distance = rel_values.leading_distance;
    for (int i = 1; i < GetNumTrajectoryTimePoints(); ++i) {
      std::tie(acc, rel_distance) =
          GetTotalAcc(observed_world, rel_values, rel_distance, dt);
      BARK_EXPECT_TRUE(!std::isnan(acc));
      // Set initial acceleration to maintain action value
      if (i == 1) {
        initial_acceleration = acc;
      }
      double angle =
          CalculateSteeringAngle(single_track, traj.row(i - 1), line,
                                 crosstrack_error_gain_, limit_steering_rate_);

      dynamic::Input input(2);
      input << acc, angle;
      traj.row(i) =
          dynamic::euler_int(*dynamic_model, traj.row(i - 1), input, dt);

      // Do not allow negative speeds
      traj(i, StateDefinition::VEL_POSITION) =
          std::max(traj(i, StateDefinition::VEL_POSITION), 0.0);
      t_i = static_cast<double>(i) * dt + start_time;
      traj(i, StateDefinition::TIME_POSITION) = t_i;

      double acc_lat = CalculateLateralAcceleration(
          single_track, angle, traj(i, StateDefinition::VEL_POSITION));
      VLOG(4) << "Plan(i=" << i << "): LonAcc:  " << acc << ", "
              << "LatAcc: " << acc_lat << ", "
              << GetAccelerationLimits();
      CheckAccelerationLimits(acc, acc_lat);
    }
  }

  return std::tuple<Trajectory, Action>(
      traj, Continuous1DAction(initial_acceleration));
}

void BehaviorIDMLaneTracking::CheckAccelerationLimits(double acc_lon,
                                                      double acc_lat) const {
  if (acc_lon > GetAccelerationLimits().lon_acc_max) {
    LOG(WARNING) << "LonAccMax is violated" << acc_lon << " vs. "
                 << GetAccelerationLimits().lon_acc_max;
  }

  if (acc_lon < GetAccelerationLimits().lon_acc_min) {
    LOG(WARNING) << "LonAccMin is violated" << acc_lon << " vs. "
                 << GetAccelerationLimits().lon_acc_min;
  }

  // Steering to the right == Accelerations are negative
  if (acc_lat < -GetAccelerationLimits().lat_acc_right_max) {
    LOG(WARNING) << "LatAccRightMax is violated" << acc_lat << " vs. "
                 << GetAccelerationLimits().lat_acc_right_max;
  }

  if (acc_lat > GetAccelerationLimits().lat_acc_left_max) {
    LOG(WARNING) << "LatAccLeftMax is violated" << acc_lat << " vs. "
                 << GetAccelerationLimits().lat_acc_left_max;
  }
}

}  // namespace behavior
}  // namespace models
}  // namespace bark
