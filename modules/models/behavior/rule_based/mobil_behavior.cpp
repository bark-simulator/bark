// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/models/behavior/idm/base_idm.hpp"
#include "modules/models/behavior/rule_based/mobil_behavior.hpp"
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


/**
 * @brief Function that chooses the LaneCorridor that has the most
 *        free-space
 * 
 * @return std::pair<LaneChangeDecision, LaneCorridorPtr> 
 */
std::pair<LaneChangeDecision, LaneCorridorPtr>
BehaviorMobilRuleBased::ChooseLaneCorridor(
    const std::vector<LaneCorridorInformation>& lane_corr_infos,
    const ObservedWorld& observed_world) const {
  auto lane_corr = observed_world.GetLaneCorridor();
  LaneChangeDecision change_decision = LaneChangeDecision::KeepLane;

  double acc_ego = 0., acc_change_ego = 0.,
         acc_behind = 0., acc_change_behind = 0.;
  std::vector<LaneCorridorInformation> all_corrs =
    ScanLaneCorridors(observed_world);
  LaneCorridorInformation lci = SelectLaneCorridor(
    all_corrs, GetLaneCorridor());
  if (lci.front.agent_info.first)
    acc_ego = CalcIDMAcc(
      lci.front.rel_distance,
      GetVelocity(observed_world.GetEgoAgent()),
      GetVelocity(lci.front.agent_info.first));
  if (lci.rear.agent_info.first)
    acc_behind = CalcIDMAcc(
      -lci.rear.rel_distance,
      GetVelocity(lci.rear.agent_info.first),
      GetVelocity(observed_world.GetEgoAgent()));

  if (lane_corr_infos.size() > 0) {
    // select corridor with most free space
    double max_advantage = -100.;
    LaneCorridorPtr tmp_lane_corr = GetLaneCorridor();
    for (const auto& li : lane_corr_infos) {
      if (li.front.agent_info.first)
        acc_change_ego = CalcIDMAcc(
          li.front.rel_distance,
          GetVelocity(observed_world.GetEgoAgent()),
          GetVelocity(li.front.agent_info.first));
      if (li.rear.agent_info.first)
        acc_change_behind = CalcIDMAcc(
          -li.rear.rel_distance,
          GetVelocity(li.rear.agent_info.first),
          GetVelocity(observed_world.GetEgoAgent()));

      double advantage_ego = acc_ego - acc_change_ego;
      double advantage_other = acc_behind - acc_change_behind;
      // acc'(ego) - acc(ego) > p [acc(behind) - acc'(behind)] + a_thr
      if (advantage_ego > politeness_*advantage_other + a_thr_ &&
          advantage_ego > max_advantage) {
        max_advantage = advantage_ego;
        tmp_lane_corr = li.lane_corridor;
      }
    }
    if (tmp_lane_corr != lane_corr && tmp_lane_corr) {
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
