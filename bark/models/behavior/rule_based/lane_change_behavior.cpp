// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/models/behavior/rule_based/lane_change_behavior.hpp"
#include <algorithm>
#include <limits>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>
#include "bark/models/behavior/idm/base_idm.hpp"
#include "bark/models/dynamic/integration.hpp"
#include "bark/models/dynamic/single_track.hpp"

namespace bark {
namespace models {
namespace behavior {

using bark::commons::transformation::FrenetPosition;
using bark::geometry::Line;
using bark::geometry::Point2d;
using bark::models::dynamic::CalculateSteeringAngle;
using bark::models::dynamic::DynamicModelPtr;
using dynamic::State;
using dynamic::StateDefinition;
using world::Agent;
using world::AgentFrenetPair;
using world::AgentId;
using world::ObservedWorld;
using world::map::LaneCorridorPtr;
using world::map::RoadCorridorPtr;
using world::objects::AgentPtr;

/**
 * @brief Calculates relative values for the ego vehicle and a given
 *        LaneCorridor
 *
 * @param observed_world ObservedWorld for the ego vehicle
 * @param lane_corr Arbitrary LaneCorridor
 * @return std::pair<AgentInformation, AgentInformation> front and rear agent
 *                                                       information
 */
std::pair<AgentInformation, AgentInformation>
BehaviorLaneChangeRuleBased::FrontRearAgents(
    const ObservedWorld& observed_world,
    const LaneCorridorPtr& lane_corr) const {
  AgentInformation front_info, rear_info;
  const auto& front_rear = observed_world.GetAgentFrontRear(lane_corr);
  const auto& ego_agent = observed_world.GetEgoAgent();
  if (front_rear.front.first) {
    // front info
    front_info.agent_info = front_rear.front;
    front_info.rel_velocity =
        GetVelocity(front_rear.front.first) - GetVelocity(ego_agent);
    front_info.rel_distance = front_rear.front.second.lon;
    front_info.is_vehicle = true;
  }
  if (front_rear.rear.first) {
    // rear info
    rear_info.agent_info = front_rear.rear;
    rear_info.rel_velocity =
        GetVelocity(front_rear.rear.first) - GetVelocity(ego_agent);
    rear_info.rel_distance = front_rear.rear.second.lon;
    rear_info.is_vehicle = true;
  } else {
    // struct has pos. values by default
    rear_info.rel_distance = -1000.;
    rear_info.rel_velocity = 0.;
  }
  return std::pair<AgentInformation, AgentInformation>(front_info, rear_info);
}

LaneCorridorInformation
BehaviorLaneChangeRuleBased::FillLaneCorridorInformation(
    const ObservedWorld& observed_world,
    const LaneCorridorPtr& lane_corr) const {
  const auto& ego_pos = observed_world.CurrentEgoPosition();  // x, y

  std::pair<AgentInformation, AgentInformation> agent_lane_info =
      FrontRearAgents(observed_world, lane_corr);
  double remaining_distance = lane_corr->LengthUntilEnd(ego_pos);
  // include distance to the end of the LaneCorridor
  agent_lane_info.first.rel_distance =
      std::min(remaining_distance, agent_lane_info.first.rel_distance);
  LaneCorridorInformation lane_corr_info(agent_lane_info.first,
                                         agent_lane_info.second, lane_corr,
                                         remaining_distance);
  return lane_corr_info;
}

/**
 * @brief Scans all LaneCorridors and composes LaneCorridorInformation
 *        that contains additional relative information
 *
 * @param observed_world ObservedWorld for vehicle
 * @return std::vector<LaneCorridorInformation> Additional LaneCorr. info
 */
std::vector<LaneCorridorInformation>
BehaviorLaneChangeRuleBased::ScanLaneCorridors(
    const ObservedWorld& observed_world) const {
  const auto& road_corr = observed_world.GetRoadCorridor();

  const auto& lane_corrs = road_corr->GetUniqueLaneCorridors();
  std::vector<LaneCorridorInformation> lane_corr_infos;
  for (const auto& lane_corr : lane_corrs) {
    LaneCorridorInformation lane_corr_info =
        FillLaneCorridorInformation(observed_world, lane_corr);
    lane_corr_infos.push_back(lane_corr_info);
  }
  return lane_corr_infos;
}

/**
 * @brief Function that chooses the LaneCorridor that has the most
 *        free-space
 *
 * @return std::pair<LaneChangeDecision, LaneCorridorPtr>
 */
std::pair<LaneChangeDecision, LaneCorridorPtr>
BehaviorLaneChangeRuleBased::ChooseLaneCorridor(
    const std::vector<LaneCorridorInformation>& lane_corr_infos,
    const ObservedWorld& observed_world) const {
  auto lane_corr = observed_world.GetLaneCorridor();
  LaneChangeDecision change_decision = LaneChangeDecision::KeepLane;
  if (lane_corr_infos.size() > 0) {
    // select corridor with most free space
    double max_rel_dist = 0.;
    LaneCorridorPtr tmp_lane_corr;
    for (const auto& li : lane_corr_infos) {
      if (li.front.rel_distance > max_rel_dist) {
        max_rel_dist = li.front.rel_distance;
        tmp_lane_corr = li.lane_corridor;
      }
    }
    if (tmp_lane_corr != lane_corr) {
      LOG(INFO) << "Agent " << observed_world.GetEgoAgentId()
                << " is changing lanes." << std::endl;
      lane_corr = tmp_lane_corr;
      change_decision = LaneChangeDecision::ChangeLane;
    }
  }
  return std::pair<LaneChangeDecision, LaneCorridorPtr>(change_decision,
                                                        lane_corr);
}

// see base class
std::pair<LaneChangeDecision, LaneCorridorPtr>
BehaviorLaneChangeRuleBased::CheckIfLaneChangeBeneficial(
    const ObservedWorld& observed_world) const {
  // as we are lazy initially we want to keep the lane
  std::vector<LaneCorridorInformation> lane_corr_infos =
      ScanLaneCorridors(observed_world);

  // find all feasible LaneCorridors by filtering
  // 1. there should be enough remaining distance left
  lane_corr_infos =
      FilterLaneCorridors(lane_corr_infos, [this](LaneCorridorInformation li) {
        return li.remaining_distance >= min_remaining_distance_;
      });
  // 2. enough space behind the ego vehicle to merge
  lane_corr_infos =
      FilterLaneCorridors(lane_corr_infos, [this](LaneCorridorInformation li) {
        return (li.rear.rel_distance <=
                -min_vehicle_rear_distance_ -
                    fabs(li.rear.rel_velocity) * time_keeping_gap_);
      });
  // 3. enough space in front of the ego vehicle to merge
  lane_corr_infos =
      FilterLaneCorridors(lane_corr_infos, [this](LaneCorridorInformation li) {
        return (li.front.rel_distance >= min_vehicle_front_distance_);
      });

  return ChooseLaneCorridor(lane_corr_infos, observed_world);
}

Trajectory BehaviorLaneChangeRuleBased::Plan(
    float delta_time, const world::ObservedWorld& observed_world) {
  using dynamic::StateDefinition;
  SetBehaviorStatus(BehaviorStatus::VALID);

  if (!observed_world.GetLaneCorridor()) {
    LOG(INFO) << "Agent " << observed_world.GetEgoAgentId()
              << ": Behavior status has expired!" << std::endl;
    SetBehaviorStatus(BehaviorStatus::EXPIRED);
    return GetLastTrajectory();
  }

  // whether to change lanes or not
  std::pair<LaneChangeDecision, LaneCorridorPtr> lane_res =
      CheckIfLaneChangeBeneficial(observed_world);

  if (lane_res.second) SetLaneCorridor(lane_res.second);

  if (!GetLaneCorridor()) {
    LOG(INFO) << "Agent " << observed_world.GetEgoAgentId()
              << ": Behavior status has expired!" << std::endl;
    SetBehaviorStatus(BehaviorStatus::EXPIRED);
    return GetLastTrajectory();
  }

  // we want to calc. the acc. based on the actual LaneCorridor
  IDMRelativeValues rel_values =
      CalcRelativeValues(observed_world, GetLaneCorridor());

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
