// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/models/behavior/rule_based/intersection_behavior.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <tuple>
#include <utility>

#include "bark/commons/params/setter_params.hpp"
#include "bark/commons/transformation/frenet.hpp"
#include "bark/models/behavior/constant_acceleration/constant_acceleration.hpp"
#include "bark/models/dynamic/integration.hpp"
#include "bark/models/dynamic/single_track.hpp"
#include "bark/world/observed_world.hpp"

namespace bark {
namespace models {
namespace behavior {

using bark::commons::SetterParams;
using bark::commons::transformation::FrenetPosition;
using bark::geometry::Norm0To2PI;
using bark::geometry::Point2d;
using bark::geometry::SignedAngleDiff;
using bark::models::behavior::BehaviorConstantAcceleration;
using bark::models::dynamic::DynamicModelPtr;
using bark::models::dynamic::State;
using bark::models::dynamic::StateDefinition;
using bark::models::dynamic::StateDefinition::THETA_POSITION;
using bark::models::dynamic::StateDefinition::VEL_POSITION;
using bark::world::AgentId;
using bark::world::AgentMap;
using bark::world::ObservedWorld;
using bark::world::ObservedWorldPtr;
using bark::world::WorldPtr;
using bark::world::objects::Agent;
using bark::world::objects::AgentPtr;
using bark::world::prediction::PredictionSettings;

/**
 * @brief Returns the first intersecting agent for the ego LaneCorr.
 *
 * @param intersecting_agents All agents that intersect the ego LaneCorr.
 * @param observed_world ObservedWorld of the vehicle
 * @return AgentPtr Agent that intersects ego LaneCorr. at earliest time
 */
std::pair<AgentId, bool> BehaviorIntersectionRuleBased::GetIntersectingAgent(
    const AgentMap& intersecting_agents,
    const ObservedWorld& observed_world) const {
  AgentId intersecting_agent_id = 0;
  bool is_intersecting = false;
  for (const auto& agent : intersecting_agents) {
    if (!observed_world.GetEgoAgent() || !agent.second)
      return std::pair<AgentId, bool>(intersecting_agent_id, false);
    const auto& road_corr = agent.second->GetRoadCorridor();
    const auto& agent_pos = agent.second->GetCurrentPosition();
    const auto& ego_state = observed_world.CurrentEgoState();
    const auto& ego_pos = observed_world.CurrentEgoPosition();
    const auto& agent_state = agent.second->GetCurrentState();
    const auto& lane_corr = road_corr->GetCurrentLaneCorridor(agent_pos);

    if (lane_corr != observed_world.GetLaneCorridor() &&
        agent.second != observed_world.GetEgoAgent() &&
        observed_world.GetLaneCorridor() != nullptr && lane_corr != nullptr) {
      // only if s of intersecting agent is larger
      double s_ego = std::get<1>(GetNearestPointAndS(
          observed_world.GetLaneCorridor()->GetCenterLine(), ego_pos));
      double s_other = std::get<1>(GetNearestPointAndS(
          observed_world.GetLaneCorridor()->GetCenterLine(), agent_pos));
      double ego_angle = Norm0To2PI(ego_state[THETA_POSITION]);
      double other_angle = Norm0To2PI(agent_state[THETA_POSITION]);
      if (fabs(ego_angle - other_angle) > angle_diff_for_intersection_ &&
          s_other > s_ego && s_other - s_ego < braking_distance_) {
        intersecting_agent_id = agent.second->GetAgentId();
        is_intersecting = true;
        break;
      }
    }
  }
  return std::pair<AgentId, bool>(intersecting_agent_id, is_intersecting);
}

/**
 * @brief Checks for intersecting agents on the ego
 *        LaneCorridor
 *
 * @param observed_world ObservedWorld
 * @param t_inc Forward delta time for the prediction
 * @return std::tuple<double, AgentPtr> time to interception and agent
 */
std::tuple<double, AgentPtr>
BehaviorIntersectionRuleBased::CheckIntersectingVehicles(
    const ObservedWorld& observed_world, double t_inc) {
  double intersection_time = 0.;
  LaneCorridorPtr lane_corr = GetLaneCorridor();
  AgentPtr lane_corr_intersecting_agent;
  // prediction
  auto params = std::make_shared<SetterParams>();
  BehaviorModelPtr prediction_model(new BehaviorConstantAcceleration(params));
  PredictionSettings prediction_settings(prediction_model, prediction_model);
  ObservedWorldPtr tmp_observed_world =
      std::dynamic_pointer_cast<ObservedWorld>(observed_world.Clone());
  tmp_observed_world->SetupPrediction(prediction_settings);

  // predict for n seconds
  for (double t = 0.; t < prediction_time_horizon_; t += prediction_t_inc_) {
    ObservedWorldPtr predicted_world = tmp_observed_world->Predict(t);
    // all agents intersecting at time t
    AgentMap intersecting_agents =
        predicted_world->GetAgentsIntersectingPolygon(
            lane_corr->GetMergedPolygon());
    // first agent intersecting
    std::pair<AgentId, bool> intersecting_agent_id =
        GetIntersectingAgent(intersecting_agents, *predicted_world);
    if (intersecting_agent_id.second) {
      lane_corr_intersecting_agent =
          observed_world.GetAgent(intersecting_agent_id.first);
      // if there is an agent that intersects at time t
      if (lane_corr_intersecting_agent) {
        intersection_time = t;
        break;
      }
    }
  }
  return std::tuple<double, AgentPtr>(intersection_time,
                                      lane_corr_intersecting_agent);
}

// see base class
Trajectory BehaviorIntersectionRuleBased::Plan(
    float delta_time, const ObservedWorld& observed_world) {
  using dynamic::StateDefinition;
  SetBehaviorStatus(BehaviorStatus::VALID);

  if (!observed_world.GetLaneCorridor()) {
    LOG(INFO) << "Agent " << observed_world.GetEgoAgentId()
              << ": Behavior status has expired!" << std::endl;
    SetBehaviorStatus(BehaviorStatus::EXPIRED);
    return GetLastTrajectory();
  }

  // check if a lane change would be beneficial
  std::pair<LaneChangeDecision, LaneCorridorPtr> lane_res =
      CheckIfLaneChangeBeneficial(observed_world);

  if (lane_res.second) SetLaneCorridor(lane_res.second);

  // check intersecting vehicles
  std::tuple<double, AgentPtr> time_agent =
      CheckIntersectingVehicles(observed_world);

  // calc. rel. values
  IDMRelativeValues rel_values =
      CalcRelativeValues(observed_world, GetLaneCorridor());

  // if there is an intersecting vehicle
  if (std::get<1>(time_agent)) {
    const auto& other_agent = std::get<1>(time_agent);
    const auto& ego_agent = observed_world.GetEgoAgent();
    const auto& other_agent_state = std::get<1>(time_agent)->GetCurrentState();
    const auto& ego_agent_state = ego_agent->GetCurrentState();
    double other_angle = Norm0To2PI(other_agent_state[THETA_POSITION]);
    double ego_angle = Norm0To2PI(ego_agent_state[THETA_POSITION]);
    double angle_diff = SignedAngleDiff(ego_angle, other_angle);

    // if there is a vehicle from the right
    if (angle_diff < 0) {
      LOG(INFO) << "Agent" << observed_world.GetEgoAgentId() << ": Agent "
                << std::get<1>(time_agent)->GetAgentId()
                << " is intersecing my corridor with " << angle_diff << "."
                << std::endl;
      rel_values.leading_distance =
          std::min(other_agent_state[VEL_POSITION] * std::get<0>(time_agent),
                   rel_values.leading_distance);
      // we want to break; set velocity to zero
      rel_values.leading_velocity = 0.;
      rel_values.has_leading_object = true;
    }
  }

  // generate traj. using rel_values
  double dt = delta_time / (GetNumTrajectoryTimePoints() - 1);
  std::tuple<Trajectory, Action> traj_action =
      GenerateTrajectory(observed_world, GetLaneCorridor(), rel_values, dt);

  // set values
  Trajectory traj = std::get<0>(traj_action);
  Action action = std::get<1>(traj_action);
  SetLastTrajectory(traj);
  SetLastAction(action);
  return traj;
}

}  // namespace behavior
}  // namespace models
}  // namespace bark
