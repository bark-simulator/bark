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

double BehaviorMobilRuleBased::CalcLongRawAccWithoutLeader(
    const LaneCorridorPtr& lane_corr, const Point2d& pos, double vel) const {
  double acc;
  double acc_free =
      BaseIDM::GetMaxAcceleration() * BaseIDM::CalcFreeRoadTerm(vel);
  if (BaseIDM::brake_lane_end_) {
    bool braking_required;
    double len_until_end;
    std::tie(braking_required, len_until_end) =
        BaseIDM::GetDistanceToLaneEnding(lane_corr, pos);
    if (braking_required) {
      acc = CalcRawIDMAcc(len_until_end, vel, 0);
    } else {
      acc = acc_free;
    }
  } else {
    acc = acc_free;
  }
  return acc;
}

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

  double acc_ego = 0;
  std::pair<LaneCorridorInformation, bool> lci_has =
      SelectLaneCorridor(lane_corr_infos, lane_corr);
  LaneCorridorInformation lci_ego;
  if (lci_has.second) {
    lci_ego = std::get<0>(lci_has);
  } else {
    // the current lane corridor is not within the available corridors
    lci_ego = FillLaneCorridorInformation(observed_world, lane_corr);
  }

  if (lci_ego.front.agent_info.first) {
    BARK_EXPECT_TRUE(lci_ego.front.rel_distance >= 0);
    acc_ego = CalcRawIDMAcc(lci_ego.front.rel_distance,
                            GetVelocity(observed_world.GetEgoAgent()),
                            GetVelocity(lci_ego.front.agent_info.first));
  } else {
    acc_ego = CalcLongRawAccWithoutLeader(
        lane_corr, observed_world.CurrentEgoPosition(),
        GetVelocity(observed_world.GetEgoAgent()));
  }

  if (!lane_corr_infos.empty()) {
    double max_advantage = -1e3;  // default val meets Incentive Criterion
    // use current lane corridor as default
    LaneCorridorPtr tmp_lane_corr = GetLaneCorridor();
    for (const auto& li : lane_corr_infos) {
      VLOG(4) << li;
      double acc_change_ego, acc_behind, acc_change_behind;
      if (li.front.agent_info.first) {
        BARK_EXPECT_TRUE(li.front.rel_distance >= 0);
        acc_change_ego = CalcRawIDMAcc(
            li.front.rel_distance, GetVelocity(observed_world.GetEgoAgent()),
            GetVelocity(li.front.agent_info.first));
      } else {
        acc_change_ego = CalcLongRawAccWithoutLeader(
            li.lane_corridor, observed_world.CurrentEgoPosition(),
            GetVelocity(observed_world.GetEgoAgent()));
      }
      // Calculating acc of new follower before changing lane
      if (li.rear.agent_info.first) {
        if (li.front.agent_info.first) {
          double distance = li.front.rel_distance - li.rear.rel_distance;
          BARK_EXPECT_TRUE(distance >= 0);
          acc_behind =
              CalcRawIDMAcc(distance, GetVelocity(li.rear.agent_info.first),
                            GetVelocity(li.front.agent_info.first));
          VLOG(4) << "Distance rear vehicle to front before: " << distance;
        } else {
          acc_behind = CalcLongRawAccWithoutLeader(
              li.lane_corridor, li.rear.agent_info.first->GetCurrentPosition(),
              GetVelocity(li.rear.agent_info.first));
        }
      } else {
        acc_behind = 0;
      }

      // Calculating acc of new follower after changing lane
      if (li.rear.agent_info.first) {
        VLOG(4) << "Distance rear vehicle to front after: "
                << -li.rear.rel_distance;
        BARK_EXPECT_TRUE(-li.rear.rel_distance >= 0);
        acc_change_behind = CalcRawIDMAcc(
            -li.rear.rel_distance, GetVelocity(li.rear.agent_info.first),
            GetVelocity(observed_world.GetEgoAgent()));
      } else {
        acc_change_behind = 0;
      }

      double advantage_ego = acc_change_ego - acc_ego;
      VLOG(4) << "advantage_ego=" << advantage_ego
              << " ... acc_change_ego=" << acc_change_ego
              << " acc_ego=" << acc_ego;
      double advantage_other = acc_behind - acc_change_behind;
      VLOG(4) << "advantage_other(aka behind)=" << advantage_other
              << " ... acc_behind=" << acc_behind
              << " acc_change_behind=" << acc_change_behind;

      // Safety Criterion:
      if (acc_change_behind <= -safe_deceleration_) {
        VLOG(4) << "Safety criterion not met." << acc_change_behind;
        continue;
      }

      // Incentive Criterion (only considering new follower!)
      // acc'(ego) - acc(ego) > p [acc(behind) - acc'(behind)] + a_thr
      if (advantage_ego > politeness_ * advantage_other + a_thr_) {
        VLOG(4) << "Incentive Criterion is met.";
        if (advantage_ego > max_advantage) {
          max_advantage = advantage_ego;
          tmp_lane_corr = li.lane_corridor;
        }
      }
    }
    if (tmp_lane_corr != lane_corr && tmp_lane_corr) {
      VLOG(4) << "Agent " << observed_world.GetEgoAgentId()
              << " is changing lanes.";
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
