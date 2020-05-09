// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/models/behavior/rule_based/simple_behavior.hpp"
#include "modules/models/behavior/idm/base_idm.hpp"
#include <algorithm>
#include <memory>
#include <utility>
#include <vector>
#include <limits>
#include <tuple>
#include "modules/models/dynamic/integration.hpp"
#include "modules/models/dynamic/single_track.hpp"

namespace modules {
namespace models {
namespace behavior {

using dynamic::State;
using dynamic::StateDefinition;
using modules::commons::transformation::FrenetPosition;
using modules::geometry::Line;
using modules::geometry::Point2d;
using modules::models::dynamic::CalculateSteeringAngle;
using modules::models::dynamic::DynamicModelPtr;
using world::Agent;
using world::AgentFrenetPair;
using world::AgentId;
using world::ObservedWorld;
using world::map::LaneCorridorPtr;
using world::map::RoadCorridorPtr;
using world::objects::AgentPtr;

std::pair<AgentInformation, AgentInformation>
BehaviorSimpleRuleBased::FrontRearAgents(
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
  return std::pair<AgentInformation, AgentInformation>(
    front_info, rear_info);
}

std::vector<LaneCorridorInformation>
BehaviorSimpleRuleBased::ScanLaneCorridors(
  const ObservedWorld& observed_world) const {
  const auto& road_corr = observed_world.GetRoadCorridor();

  const auto& lane_corrs = road_corr->GetUniqueLaneCorridors();
  const auto& ego_pos = observed_world.CurrentEgoPosition();  // x, y
  std::vector<LaneCorridorInformation> lane_corr_infos;
  for (const auto& lane_corr : lane_corrs) {
    // all the informations we need
    LaneCorridorInformation lane_corr_info;
    std::pair<AgentInformation, AgentInformation> agent_lane_info =
      FrontRearAgents(observed_world, lane_corr);
    double remaining_distance = lane_corr->LengthUntilEnd(ego_pos);
    lane_corr_info.front = agent_lane_info.first;
    lane_corr_info.rear = agent_lane_info.second;
    lane_corr_info.remaining_distance = remaining_distance;
    lane_corr_info.lane_corridor = lane_corr;
    lane_corr_infos.push_back(lane_corr_info);
  }
  return lane_corr_infos;
}

std::pair<LaneChangeDecision, LaneCorridorPtr>
BehaviorSimpleRuleBased::CheckIfLaneChangeBeneficial(
  const ObservedWorld& observed_world) const {
  // as we are lazy initially we want to keep the lane
  auto lane_corr = observed_world.GetLaneCorridor();
  LaneChangeDecision change_decision = LaneChangeDecision::KeepLane;
  std::vector<LaneCorridorInformation> lane_corr_infos =
    ScanLaneCorridors(observed_world);

  // find all feasible LaneCorridors by filtering
  // 1. there should be enough remaining distance left
  lane_corr_infos =
    FilterLaneCorridors(
      lane_corr_infos,
      [this](LaneCorridorInformation li) {
        return li.remaining_distance >= min_remaining_distance_; });
  // 2. enough space behind the ego vehicle to merge
  lane_corr_infos =
    FilterLaneCorridors(
      lane_corr_infos,
      [this](LaneCorridorInformation li) {
        return (
          li.rear.rel_distance <=
          -min_vehicle_rear_distance_-li.rear.rel_velocity*time_keeping_gap_);
      });
  // 3. enough space in front of the ego vehicle to merge
  lane_corr_infos =
    FilterLaneCorridors(
      lane_corr_infos,
      [this](LaneCorridorInformation li) {
        return (
          li.front.rel_distance >=
          min_vehicle_front_distance_+li.front.rel_velocity*time_keeping_gap_);
        });

  // TODO(@hart): mobil equation instead of free space
  // acc'(ego) - acc(ego) > p [acc(behind) - acc'(behind)] + a_thr -> change lanes
  // acc'(ego): CalcIDMAcc(li.front.rel_distance, vel_ego, GetVelocity(li.front.agent))
  // acc'(behind): CalcIDMAcc(-li.rear.rel_distance, GetVelocity(li.front.agent), vel_ego)
  // acc(ego): CalcIDMAcc(li_ego.front.rel_distance, vel_ego, GetVelocity(font_agent))
  // acc(behind): CalcIDMAcc(-li_ego.rear.rel_distance, GetVelocity(li.front.agent), vel_ego)
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
    }
  }

  return std::pair<LaneChangeDecision, LaneCorridorPtr>(
    change_decision, lane_corr);
}

}  // namespace behavior
}  // namespace models
}  // namespace modules
