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

  Input initial_input;
  if (!line.obj_.empty()) {
    // adding state at t=0
    traj.block<1, StateDefinition::MIN_STATE_SIZE>(0, 0) =
        ego_vehicle_state.transpose().block<1, StateDefinition::MIN_STATE_SIZE>(
            0, 0);
    traj(0, StateDefinition::TIME_POSITION) = start_time;

    double rel_distance = rel_values.leading_distance;
    double acc = 0.;
    for (int i = 1; i < GetNumTrajectoryTimePoints(); ++i) {
      std::tie(acc, rel_distance) =
          GetTotalAcc(observed_world, rel_values, rel_distance, dt);
      BARK_EXPECT_TRUE(!std::isnan(acc));
      double angle =
          CalculateSteeringAngle(single_track, traj.row(i - 1), line,
                                 crosstrack_error_gain_, limit_steering_rate_);

      dynamic::Input input(2);
      input << acc, angle;
      traj.row(i) =
          dynamic::euler_int(*dynamic_model, traj.row(i - 1), input, dt);

      // Restrict allowed speed
      traj(i, StateDefinition::VEL_POSITION) =
          std::max(traj(i, StateDefinition::VEL_POSITION), GetMinVelocity());
      double t_i = static_cast<double>(i) * dt + start_time;
      traj(i, StateDefinition::TIME_POSITION) = t_i;

      double acc_lat = CalculateLateralAcceleration(
          single_track, angle, traj(i, StateDefinition::VEL_POSITION));
      // VLOG(4) << "Plan(i=" << i << "): LonAcc:  " << acc << ", "
      //         << "LatAcc: " << acc_lat << ", "
      //         << GetAccelerationLimits();
      CheckAccelerationLimits(acc, acc_lat);

      // save initial input
      if (i == 1) {
        initial_input = input;
      }
    }
  }

  return std::tuple<Trajectory, Action>(traj, initial_input);
}

void BehaviorIDMLaneTracking::CheckAccelerationLimits(double acc_lon,
                                                      double acc_lat) const {
  const auto almost_smaller = [](const double val1, const double val2) {
    const double precision = 0.1;
    return val1 <= val2 || std::abs(val1 - val2)  < precision;
  };
  const auto almost_larger = [](const double val1, const double val2) {
    const double precision = 0.1;
    return val1 >= val2 || std::abs(val1 - val2)  < precision;
  };
  if (acc_lon >= 0 && !almost_smaller(acc_lon, GetAccelerationLimits().lon_acc_max)) {
    VLOG(3) << "LonAccMax is violated " << acc_lon << " vs. "
                 << GetAccelerationLimits().lon_acc_max;
  }

  if (acc_lon < 0 && !almost_larger(acc_lon, GetAccelerationLimits().lon_acc_min)) {
    VLOG(3) << "LonAccMin is violated" << acc_lon << " vs. "
                 << GetAccelerationLimits().lon_acc_min;
  }

  // Steering to the right == Accelerations are negative
  if (acc_lat <= 0 && !almost_larger(acc_lat, GetAccelerationLimits().lat_acc_min)) {
    VLOG(3) << "LatAccRightMax is violated " << acc_lat << " vs. "
                 << GetAccelerationLimits().lat_acc_min;
  }

  if (acc_lat > 0 && !almost_smaller(acc_lat, GetAccelerationLimits().lat_acc_max)) {
    VLOG(3) << "LatAccLeftMax is violated " << acc_lat << " vs. "
                 << GetAccelerationLimits().lat_acc_max;
  }
}

}  // namespace behavior
}  // namespace models
}  // namespace bark
