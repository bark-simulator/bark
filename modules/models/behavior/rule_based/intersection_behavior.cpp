// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/models/behavior/rule_based/intersection_behavior.hpp"

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
#include "modules/models/behavior/constant_velocity/constant_velocity.hpp"

namespace modules {
namespace models {
namespace behavior {

using modules::commons::transformation::FrenetPosition;
using modules::geometry::Point2d;
using modules::models::dynamic::State;
using modules::models::dynamic::StateDefinition;
using modules::world::objects::Agent;
using modules::world::AgentMap;
using modules::world::objects::AgentPtr;
using modules::world::WorldPtr;
using modules::world::ObservedWorld;
using modules::models::dynamic::DynamicModelPtr;
using modules::models::behavior::BehaviorConstantVelocity;
using modules::world::prediction::PredictionSettings;

std::pair<double, bool> BehaviorIntersectionRuleBased::CheckIntersectingVehicles(
  const LaneCorridorPtr& lane_corr,
  const ObservedWorld& observed_world,
  double pred_horizon,
  double t_inc) {
  bool intersecting_agent = false;
  double augmented_distance = 0.;

  // prediction
  BehaviorModelPtr prediction_model(new BehaviorConstantVelocity(GetParams()));
  PredictionSettings prediction_settings(prediction_model, prediction_model);
  // because const Obs.World
  ObservedWorld tmp_observed_world = observed_world;
  tmp_observed_world.SetupPrediction(prediction_settings);
  double intersection_time = 0., vel_other = 0.;

  // predict for n seconds
  for (double t = 0.; t < pred_horizon; t *= t_inc) {
    WorldPtr predicted_world = tmp_observed_world.Predict(t);
    AgentMap intersecting_agents = tmp_observed_world.GetAgentsIntersectingPolygon(
      lane_corr->GetMergedPolygon());
    // take lowest time
    if (intersecting_agents.size() > 0) {
      if (intersecting_agents[0] != tmp_observed_world.GetEgoAgent()) {
        intersection_time = t;
        vel_other = BehaviorSimpleRuleBased::GetVelocity(
          intersecting_agents[0]);
        intersecting_agent = true;
        break;
      }
    }
  }
  if (intersecting_agent) {
    augmented_distance = vel_other*intersection_time;
    return std::pair<double, bool>(augmented_distance, intersecting_agent);
  }
  return std::pair<double, bool>(augmented_distance, intersecting_agent);
}

//! IDM Model will assume other front vehicle as constant velocity during
Trajectory BehaviorIntersectionRuleBased::Plan(
    float delta_time, const ObservedWorld& observed_world) {
  using dynamic::StateDefinition;
  SetBehaviorStatus(BehaviorStatus::VALID);

  // whether to change lanes or not
  std::pair<LaneChangeDecision, LaneCorridorPtr> lane_res =
    CheckIfLaneChangeBeneficial(observed_world);
  SetLaneCorridor(lane_res.second);

  if (!GetLaneCorridor()) {
    return GetLastTrajectory();
  }

  // check intersecting vehicles
  std::pair<double, bool> dist_isinter = CheckIntersectingVehicles(
    GetLaneCorridor(),
    observed_world);

  // we want to calc. the acc. based on the actual LaneCorridor
  std::tuple<double, double, bool> rel_values = CalcRelativeValues(
    observed_world,
    GetLaneCorridor());

  // if there is an intersecting vehicle
  if (dist_isinter.second) {
    // TODO(@hart): integrate logic for breaking
  }

  std::tuple<Trajectory, Action> traj_action =
    GenerateTrajectory(
      observed_world, GetLaneCorridor(), rel_values, delta_time);

  // set values
  Trajectory traj = std::get<0>(traj_action);
  Action action = std::get<1>(traj_action);
  SetLastTrajectory(traj);
  SetLastAction(action);
  return traj;
}

}  // namespace behavior
}  // namespace models
}  // namespace modules
