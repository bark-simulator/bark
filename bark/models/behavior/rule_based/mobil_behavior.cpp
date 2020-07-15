// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/models/behavior/rule_based/mobil_behavior.hpp"
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
 * @brief Function that chooses the LaneCorridor according to the
 *        MOBIL model
 *
 * @return std::pair<LaneChangeDecision, LaneCorridorPtr>
 */
std::pair<LaneChangeDecision, LaneCorridorPtr>
BehaviorMobilRuleBased::ChooseLaneCorridor(
    const std::vector<LaneCorridorInformation>& lane_corr_infos,
    const ObservedWorld& observed_world) const {
  auto lane_corr = observed_world.GetLaneCorridor();
  LaneChangeDecision change_decision = LaneChangeDecision::KeepLane;

  double acc_ego;
  std::pair<LaneCorridorInformation, bool> lci_has =
      SelectLaneCorridor(lane_corr_infos, GetLaneCorridor());
  if (std::get<1>(lci_has)) {
    LaneCorridorInformation lci = std::get<0>(lci_has);
    if (lci.front.agent_info.first) {
      acc_ego = CalcIDMAcc(lci.front.rel_distance,
                           GetVelocity(observed_world.GetEgoAgent()),
                           GetVelocity(lci.front.agent_info.first));
    } else {
      // we assume that soon ending lane corridors are filtered out anyway
      acc_ego =
          BaseIDM::GetMaxAcceleration() *
          BaseIDM::CalcFreeRoadTerm(GetVelocity(observed_world.GetEgoAgent()));
    }
  } else {
    LOG(ERROR) << "No ego lane corridor set" << std::endl;
  }

  if (lane_corr_infos.size() > 0) {
    double max_advantage = -100.;
    // use current lane corridor as default
    LaneCorridorPtr tmp_lane_corr = GetLaneCorridor();
    for (const auto& li : lane_corr_infos) {
      // LOG(INFO) << li << std::endl;
      double acc_change_ego, acc_behind, acc_change_behind;
      if (li.front.agent_info.first) {
        acc_change_ego = CalcIDMAcc(li.front.rel_distance,
                                    GetVelocity(observed_world.GetEgoAgent()),
                                    GetVelocity(li.front.agent_info.first));
      } else {
        // we assume that soon ending lane corridors are filtered out anyway
        acc_change_ego = BaseIDM::GetMaxAcceleration() *
                         BaseIDM::CalcFreeRoadTerm(
                             GetVelocity(observed_world.GetEgoAgent()));
      }
      // Calculating acc of new follower before changing lane
      if (li.rear.agent_info.first) {
        if (li.front.agent_info.first) {
          double distance = li.front.rel_distance - li.rear.rel_distance;
          acc_behind =
              CalcIDMAcc(distance, GetVelocity(li.rear.agent_info.first),
                         GetVelocity(li.front.agent_info.first));
          // LOG(INFO) << "Distance rear vehicle to front before: " << distance
          //           << std::endl;
        } else {
          // TODO: take care of ending lane corridor!
          acc_behind =
              BaseIDM::GetMaxAcceleration() *
              BaseIDM::CalcFreeRoadTerm(GetVelocity(li.rear.agent_info.first));
        }
      } else {
        acc_behind = 0;
      }
      
      // Calculating acc of new follower after changing lane
      if (li.rear.agent_info.first) {
        // LOG(INFO) << "Distance rear vehicle to front after: "
        //           << -li.rear.rel_distance << std::endl;
        acc_change_behind = CalcIDMAcc(
            -li.rear.rel_distance, GetVelocity(li.rear.agent_info.first),
            GetVelocity(observed_world.GetEgoAgent()));
      } else {
        acc_change_behind = 0;
      }

      double advantage_ego = acc_change_ego - acc_ego;
      // LOG(INFO) << "advantage_ego=" << advantage_ego
      //           << " ... acc_change_ego=" << acc_change_ego
      //           << " acc_ego=" << acc_ego << std::endl;
      double advantage_other = acc_behind - acc_change_behind;
      // LOG(INFO) << "advantage_other(aka behind)=" << advantage_other
      //           << " ... acc_behind=" << acc_behind
      //           << " acc_change_behind=" << acc_change_behind << std::endl;

      // Safety Criterion:
      if (acc_change_behind <= -b_safe_) {
        // LOG(INFO) << "Safety criterion not met." << acc_change_behind
        //           << std::endl;
        continue;
      }

      // Incentive Criterion (only condiering new follower!)
      // acc'(ego) - acc(ego) > p [acc(behind) - acc'(behind)] + a_thr
      if (advantage_ego > politeness_ * advantage_other + a_thr_) {
        // LOG(INFO) << "Incentive Criterion is met." << std::endl;
        if (advantage_ego > max_advantage) {
          max_advantage = advantage_ego;
          tmp_lane_corr = li.lane_corridor;
        }
      }
    }
    if (tmp_lane_corr != lane_corr && tmp_lane_corr) {
      // LOG(INFO) << "Agent " << observed_world.GetEgoAgentId()
      //           << " is changing lanes." << std::endl;
      lane_corr = tmp_lane_corr;
      change_decision = LaneChangeDecision::ChangeLane;
    }
  }

  return std::pair<LaneChangeDecision, LaneCorridorPtr>(change_decision,
                                                        lane_corr);
}

}  // namespace behavior
}  // namespace models
}  // namespace bark
