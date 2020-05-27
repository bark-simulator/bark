// Copyright (c) 2020 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/models/behavior/idm/idm_lane_tracking.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>
#include <tuple>

#include "modules/commons/transformation/frenet.hpp"
#include "modules/world/observed_world.hpp"
#include "modules/models/dynamic/single_track.hpp"
#include "modules/models/dynamic/integration.hpp"

namespace modules {
namespace models {
namespace behavior {

using modules::commons::transformation::FrenetPosition;
using modules::geometry::Point2d;
using modules::models::dynamic::State;
using modules::models::dynamic::StateDefinition;
using modules::world::objects::Agent;
using modules::world::objects::AgentPtr;
using modules::models::dynamic::DynamicModelPtr;


std::tuple<Trajectory, Action> BehaviorIDMLaneTracking::GenerateTrajectory(
    const world::ObservedWorld& observed_world,
    const LaneCorridorPtr& lane_corr,
    const IDMRelativeValues& rel_values,
    double dt) const {
  // definitions
  const DynamicModelPtr dynamic_model =
    observed_world.GetEgoAgent()->GetDynamicModel();
  auto single_track =
    std::dynamic_pointer_cast<dynamic::SingleTrackModel>(dynamic_model);
  if (!single_track) {
    LOG(FATAL) << "Only SingleTrack as dynamic model supported!";
  }
  dynamic::State ego_vehicle_state = observed_world.CurrentEgoState();
  double t_i = 0., acc = 0.;
  geometry::Line line = lane_corr->GetCenterLine();
  dynamic::Trajectory traj(GetNumTrajectoryTimePoints(),
                           static_cast<int>(StateDefinition::MIN_STATE_SIZE));

  if (!line.obj_.empty()) {
    // adding state at t=0
    traj.block<1, StateDefinition::MIN_STATE_SIZE>(0, 0) =
      ego_vehicle_state.transpose().block<1, StateDefinition::MIN_STATE_SIZE>(
        0, 0);
    float vel_i = ego_vehicle_state(StateDefinition::VEL_POSITION);

    double rel_distance = rel_values.leading_distance;
    for (int i = 1; i < GetNumTrajectoryTimePoints(); ++i) {
      std::tie(acc, rel_distance) =
        GetTotalAcc(observed_world, rel_values, rel_distance, dt);
      BARK_EXPECT_TRUE(!std::isnan(acc));
      double angle = CalculateSteeringAngle(
        single_track, traj.row(i - 1), line, crosstrack_error_gain_,
        limit_steering_rate_);

      dynamic::Input input(2);
      input << acc, angle;
      traj.row(i) =
        dynamic::euler_int(*dynamic_model, traj.row(i - 1), input, dt);
      // Do not allow negative speeds
      traj(i, StateDefinition::VEL_POSITION) =
          std::max(traj(i, StateDefinition::VEL_POSITION), 0.0f);
    }
  }
  Action action(acc);
  return std::tuple<Trajectory, Action>(traj, action);
}



}  // namespace behavior
}  // namespace models
}  // namespace modules
