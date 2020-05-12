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
#include "modules/commons/params/default_params.hpp"
#include "modules/models/behavior/constant_velocity/constant_velocity.hpp"

namespace modules {
namespace models {
namespace behavior {

using modules::commons::transformation::FrenetPosition;
using modules::commons::DefaultParams;
using modules::geometry::Point2d;
using modules::geometry::SignedAngleDiff;
using modules::models::dynamic::State;
using modules::models::dynamic::StateDefinition;
using modules::models::dynamic::StateDefinition::THETA_POSITION;
using modules::models::dynamic::StateDefinition::VEL_POSITION;
using modules::world::objects::Agent;
using modules::world::AgentMap;
using modules::world::objects::AgentPtr;
using modules::world::WorldPtr;
using modules::world::AgentId;
using modules::world::ObservedWorld;
using modules::world::ObservedWorldPtr;
using modules::models::dynamic::DynamicModelPtr;
using modules::models::behavior::BehaviorConstantVelocity;
using modules::world::prediction::PredictionSettings;
using modules::geometry::Norm0To2PI;


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
      double s_other = std::get<1>(
        GetNearestPointAndS(observed_world.GetLaneCorridor()->GetCenterLine(),
        agent_pos));
      double ego_angle = Norm0To2PI(ego_state[THETA_POSITION]);
      double other_angle = Norm0To2PI(agent_state[THETA_POSITION]);
      if (fabs(ego_angle - other_angle) > angle_diff_for_intersection_ &&
          s_other > s_ego &&
          s_other - s_ego < braking_distance_) {
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
  const ObservedWorld& observed_world,
  double t_inc) {
  double intersection_time = 0.;
  LaneCorridorPtr lane_corr = GetLaneCorridor();
  AgentPtr lane_corr_intersecting_agent;
  // prediction
  auto params = std::make_shared<DefaultParams>();
  BehaviorModelPtr prediction_model(new BehaviorConstantVelocity(params));
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
    std::pair<AgentId, bool> intersecting_agent_id = GetIntersectingAgent(
        intersecting_agents,
        *predicted_world);
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
  return std::tuple<double, AgentPtr>(
    intersection_time, lane_corr_intersecting_agent);
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
  SetLaneCorridor(lane_res.second);

  // if there is no lane_corr to be chosen
  if (!observed_world.GetLaneCorridor() && !lane_res.second) {
    LOG(INFO) << "Agent " << observed_world.GetEgoAgentId()
              << ": Behavior status has expired!" << std::endl;
    SetBehaviorStatus(BehaviorStatus::EXPIRED);
    return GetLastTrajectory();
  }

  // check intersecting vehicles
  std::tuple<double, AgentPtr> time_agent = CheckIntersectingVehicles(
    observed_world);

  // calc. rel. values
  std::tuple<double, double, bool> rel_values = CalcRelativeValues(
    observed_world,
    GetLaneCorridor());

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
      LOG(INFO) << "Agent" << observed_world.GetEgoAgentId()
                << ": Agent " << std::get<1>(time_agent)->GetAgentId()
                << " is intersecing my corridor with "
                << angle_diff << "."<< std::endl;
      std::get<0>(rel_values) =
        std::min(
          other_agent_state[VEL_POSITION]*std::get<0>(time_agent),
          std::get<0>(rel_values));
      // we want to break; set velocity to zero
      std::get<1>(rel_values) = 0.;
      std::get<2>(rel_values) = true;
    }
  }

  // generate traj. using rel_values
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
