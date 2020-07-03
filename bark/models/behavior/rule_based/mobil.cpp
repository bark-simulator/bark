// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/models/behavior/rule_based/mobil.hpp"
#include <algorithm>
#include <memory>
#include <utility>
#include "bark/models/behavior/idm/base_idm.hpp"
#include "bark/models/behavior/rule_based/lane_change_behavior.hpp"
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
using StateDefinition::VEL_POSITION;
using world::Agent;
using world::AgentFrenetPair;
using world::AgentId;
using world::ObservedWorld;
using world::map::LaneCorridorPtr;
using world::objects::AgentPtr;

double BehaviorMobil::CalcNetDistanceFromFrenet(
    const std::shared_ptr<const Agent>& ego_agent,
    const FrenetPosition& ego_frenet,
    const std::shared_ptr<const Agent>& leading_agent,
    const FrenetPosition& leading_frenet) const {
  const float vehicle_length =
      ego_agent->GetShape().front_dist_ + leading_agent->GetShape().rear_dist_;
  const double net_distance =
      leading_frenet.lon - vehicle_length - ego_frenet.lon;
  return net_distance;
}

double BehaviorMobil::CalcLongRawAccWithoutLeader(
    const world::LaneCorridorPtr& lane_corridor,
    const bark::geometry::Point2d& pos, const float vel) const {
  double acc;
  if (stop_at_lane_ending_) {
    // TODO(@hart): change to parameter
    const double net_distance = lane_corridor->LengthUntilEnd(pos) - 15.0;
    // setting vel_other to zero
    acc = BaseIDM::CalcRawIDMAcc(net_distance, vel, 0.0);
  } else {
    acc = BaseIDM::GetMaxAcceleration() * BaseIDM::CalcFreeRoadTerm(vel);
  }
  return acc;
}

std::pair<LaneChangeDecision, LaneCorridorPtr>
BehaviorMobil::CheckIfLaneChangeBeneficial(
    const ObservedWorld& observed_world) {
  using world::FrontRearAgents;

  std::shared_ptr<const Agent> ego_agent = observed_world.GetEgoAgent();
  FrenetPosition frenet_ego = ego_agent->CurrentFrenetPosition();

  const float vel_ego = ego_agent->GetCurrentState()(VEL_POSITION);
  const LaneCorridorPtr current_corridor = observed_world.GetLaneCorridor();

  FrontRearAgents agents_current_lane = observed_world.GetAgentFrontRear();
  std::pair<AgentPtr, FrenetPosition> follower_current =
      agents_current_lane.rear;
  std::pair<AgentPtr, FrenetPosition> leader_current =
      agents_current_lane.front;

  // [STEP 1] acceleration of ego vehicle (before ... in the present situation)
  double acc_c_before;
  // ---------------------------------------------------
  // >>>>  [ego_agent] >>>> [leader_current] >>>>
  // ---------------------------------------------------
  if (leader_current.first) {
    const double dist_c_before =
        CalcNetDistanceFromFrenet(ego_agent, frenet_ego, leader_current.first,
                                  leader_current.second + frenet_ego);
    acc_c_before = BaseIDM::CalcRawIDMAcc(
        dist_c_before, vel_ego,
        leader_current.first->GetCurrentState()(VEL_POSITION));

  } else {
    // acc_c_before = GetMaxAcceleration() * CalcFreeRoadTerm(vel_ego);
    acc_c_before = CalcLongRawAccWithoutLeader(
        current_corridor, ego_agent->GetCurrentPosition(), vel_ego);
  }

  // [STEP 2] acceleration of old follower (before & after a prospective lane
  // change)
  double acc_o_before, acc_o_after;
  if (follower_current.first) {  // there is a follower
    // ---------------------------------------------------
    // >>>>  [follower_current] >>>> [ego_agent] >>>>
    // ---------------------------------------------------
    const double dist_o_before = CalcNetDistanceFromFrenet(
        follower_current.first, follower_current.second + frenet_ego, ego_agent,
        frenet_ego);
    acc_o_before = BaseIDM::CalcRawIDMAcc(
        dist_o_before, follower_current.first->GetCurrentState()(VEL_POSITION),
        vel_ego);
    // ---------------------------------------------------
    // >>>>  [follower_current] >>>> [leader_current] >>>>
    // ---------------------------------------------------
    const double vel_follower_current =
        follower_current.first->GetCurrentState()(VEL_POSITION);
    if (leader_current.first) {
      const double dist_o_after = CalcNetDistanceFromFrenet(
          follower_current.first, follower_current.second + frenet_ego,
          leader_current.first, leader_current.second + frenet_ego);
      acc_o_after = BaseIDM::CalcRawIDMAcc(
          dist_o_after, vel_follower_current,
          leader_current.first->GetCurrentState()(VEL_POSITION));
    } else {
      // acc_o_after = GetMaxAcceleration() *
      // CalcFreeRoadTerm(follower_current.first->GetCurrentState()(VEL_POSITION));
      acc_o_after = CalcLongRawAccWithoutLeader(
          current_corridor, follower_current.first->GetCurrentPosition(),
          vel_follower_current);
    }
  } else {
    acc_o_before = 0, acc_o_after = 0;
  }
  double acc_threshold = acceleration_threshold_;
  const Point2d ego_pos = observed_world.CurrentEgoPosition();

  auto road_corridor = observed_world.GetRoadCorridor();
  LaneCorridorPtr left_corridor;
  LaneCorridorPtr right_corridor;
  std::tie(left_corridor, right_corridor) =
      road_corridor->GetLeftRightLaneCorridor(ego_pos);

  AgentId ego_agent_id = observed_world.GetEgoAgentId();

  if (right_corridor) {
    //! Right lane exists
    FrontRearAgents agents_right_lane =
        observed_world.GetAgentFrontRearForId(ego_agent_id, right_corridor);
    std::pair<AgentPtr, FrenetPosition> follower_right = agents_right_lane.rear;
    std::pair<AgentPtr, FrenetPosition> leader_right = agents_right_lane.front;

    // [STEP 3] acceleration of ego vehicle (after a prospective lane change)
    double acc_c_after;
    if (leader_right.first) {
      // ---------------------------------------------------
      // >>>>  [ego_agent] >>>> [leader_right] >>>>
      // ---------------------------------------------------
      const double dist_c_after =
          CalcNetDistanceFromFrenet(ego_agent, frenet_ego, leader_right.first,
                                    leader_right.second + frenet_ego);
      acc_c_after = BaseIDM::CalcRawIDMAcc(
          dist_c_after, vel_ego,
          leader_right.first->GetCurrentState()(VEL_POSITION));

    } else {
      // acc_c_after = GetMaxAcceleration() * CalcFreeRoadTerm(vel_ego);
      acc_c_after = CalcLongRawAccWithoutLeader(
          right_corridor, ego_agent->GetCurrentPosition(), vel_ego);
    }

    // TODO(@Klemens): is this correct?
    if (leader_current.first && asymmetric_passing_rules_) {
      float vel_leader_left =
          leader_current.first->GetCurrentState()(VEL_POSITION);

      // Passing on the right is disallowed, therefore if the ego vehicle is
      // faster than the leading vehicle on the left lane, its acceleration on
      // the right lane acc_c_after cannot be higher than its acceleration on
      // the left lane acc_c_before
      if (vel_ego > vel_leader_left && vel_leader_left > critical_velocity_) {
        acc_c_after = std::min(acc_c_before, acc_c_after);
      }
    }

    double acc_n_before, acc_n_after;
    if (follower_right.first) {
      // [STEP 4] acceleration of new follower (before ... in the present
      // situation)
      // ---------------------------------------------------
      // >>>>  [follower_right] >>>> [leader_right] >>>>
      // ---------------------------------------------------
      const double vel_follower_right =
          follower_right.first->GetCurrentState()(VEL_POSITION);
      if (leader_right.first) {
        const double dist_n_before = CalcNetDistanceFromFrenet(
            follower_right.first, follower_right.second + frenet_ego,
            leader_right.first, leader_right.second + frenet_ego);
        acc_n_before = BaseIDM::CalcRawIDMAcc(
            dist_n_before, vel_follower_right,
            leader_right.first->GetCurrentState()(VEL_POSITION));
      } else {
        // acc_n_before = GetMaxAcceleration() *
        // CalcFreeRoadTerm(vel_follower_right);
        acc_n_before = CalcLongRawAccWithoutLeader(
            right_corridor, follower_right.first->GetCurrentPosition(),
            vel_follower_right);
      }
      // [STEP 5] acceleration of new follower (after a prospective lane
      // change)
      // ---------------------------------------------------
      // >>>>  [follower_right] >>>> [ego_agent] >>>>
      // ---------------------------------------------------
      const double dist_n_after = CalcNetDistanceFromFrenet(
          follower_right.first, follower_right.second + frenet_ego, ego_agent,
          frenet_ego);
      acc_n_after =
          BaseIDM::CalcRawIDMAcc(dist_n_after, vel_follower_right, vel_ego);
    } else {
      acc_n_before = 0, acc_n_after = 0;
    }

    // Safety criterion ensures that the after lane change, the new follower
    // can still safely decelerate to avoid a crash
    if (acc_n_after >= -safe_deceleration_) {
      double benefit;
      if (asymmetric_passing_rules_) {
        // Neglect the disadvantage of the following vehicle on the right lane
        // (acc_n_after - acc_n_before), because the left lane has priority
        benefit = acc_c_after - acc_c_before +
                  politeness_ * (acc_o_after - acc_o_before);
        // std::cout << " to right: " << acc_c_after << " - " << acc_c_before
        // << " + " << politeness_ << " * (" << acc_o_after << " - " <<
        // acc_o_before << ")"; std::cout << " = " << benefit << " > " <<
        // acc_threshold - acceleration_bias_ << std::endl;
      } else {
        benefit = acc_c_after - acc_c_before +
                  politeness_ *
                      (acc_n_after - acc_n_before + acc_o_after - acc_o_before);
      }
      double threshold = acc_threshold;
      if (asymmetric_passing_rules_) {
        threshold -= acceleration_bias_;
      }

      // Advantage criterion
      if (benefit > threshold) {
        if (asymmetric_passing_rules_) {
          // std::cout << "Go right. Ego improves by " << acc_c_after -
          // acc_c_before; std::cout << ", others improve by " << acc_n_after
          // - acc_n_before + acc_o_after - acc_o_before << std::endl;
        } else {
          // std::cout << "Go right. Ego improves by " << acc_c_after -
          // acc_c_before; std::cout << ", others improve by " << acc_o_after
          // - acc_o_before << std::endl;
        }
        acc_threshold = benefit;
        return std::make_pair(LaneChangeDecision::ChangeRight, right_corridor);
      }
    } else {
      // std::cout << " To right would be unsafe: " << acc_n_after << " >= "
      // << -safe_decel_ << std::endl;
    }
  }

  if (left_corridor) {
    //! Left Lane exists

    FrontRearAgents agents_left_lane =
        observed_world.GetAgentFrontRearForId(ego_agent_id, left_corridor);
    std::pair<AgentPtr, FrenetPosition> follower_left = agents_left_lane.rear;
    std::pair<AgentPtr, FrenetPosition> leader_left = agents_left_lane.front;

    // [STEP 3] acceleration of ego vehicle (after a prospective lane change)
    double acc_c_after;
    if (leader_left.first) {
      // ---------------------------------------------------
      // >>>>  [ego_agent] >>>> [leader_left] >>>>
      // ---------------------------------------------------
      const double dist_c_after =
          CalcNetDistanceFromFrenet(ego_agent, frenet_ego, leader_left.first,
                                    leader_left.second + frenet_ego);
      acc_c_after = BaseIDM::CalcRawIDMAcc(
          dist_c_after, vel_ego,
          leader_left.first->GetCurrentState()(VEL_POSITION));
    } else {
      // acc_c_after = GetMaxAcceleration() * CalcFreeRoadTerm(vel_ego);
      acc_c_after = CalcLongRawAccWithoutLeader(
          left_corridor, ego_agent->GetCurrentPosition(), vel_ego);
    }

    // TODO(@Klemens): is this correct?
    if (leader_left.first && asymmetric_passing_rules_) {
      float vel_leader_left =
          leader_left.first->GetCurrentState()(VEL_POSITION);
      // Passing on the right is disallowed, therefore if the ego vehicle is
      // faster than the leading vehicle on the left lane, its acceleration on
      // the right lane acc_c_before cannot be higher than its acceleration on
      // the right lane acc_c_after
      if (vel_ego > vel_leader_left && vel_leader_left > critical_velocity_) {
        acc_c_before = std::min(acc_c_before, acc_c_after);
      }
    }

    double acc_n_before, acc_n_after;
    if (follower_left.first) {
      // [STEP 4] acceleration of new follower (before ... in the present
      // situation)
      // ---------------------------------------------------
      // >>>>  [follower_left] >>>> [leader_left] >>>>
      // ---------------------------------------------------
      const double vel_follower_left =
          follower_left.first->GetCurrentState()(VEL_POSITION);
      if (leader_left.first) {
        const double dist_n_before = CalcNetDistanceFromFrenet(
            follower_left.first, follower_left.second + frenet_ego,
            leader_left.first, leader_left.second + frenet_ego);
        acc_n_before = BaseIDM::CalcRawIDMAcc(
            dist_n_before, vel_follower_left,
            leader_left.first->GetCurrentState()(VEL_POSITION));
      } else {
        // acc_n_before = GetMaxAcceleration() *
        // CalcFreeRoadTerm(vel_follower_left);
        acc_n_before = CalcLongRawAccWithoutLeader(
            left_corridor, follower_left.first->GetCurrentPosition(),
            vel_follower_left);
      }
      // acceleration of new follower (after a prospective lane change)
      // ---------------------------------------------------
      // >>>>  [follower_left] >>>> [ego_agent] >>>>
      // ---------------------------------------------------
      const double dist_n_after = CalcNetDistanceFromFrenet(
          follower_left.first, follower_left.second + frenet_ego, ego_agent,
          frenet_ego);

      acc_n_after =
          BaseIDM::CalcRawIDMAcc(dist_n_after, vel_follower_left, vel_ego);
    } else {
      acc_n_before = 0, acc_n_after = 0;
    }

    // Safety criterion ensures that the after lane change, the new follower
    // can still safely decelerate to avoid a crash
    if (acc_n_after >= -safe_deceleration_) {
      double benefit;
      if (asymmetric_passing_rules_) {
        // Neglect the disadvantage of the following vehicle on the right lane
        // (acc_o_after - acc_o_before), because the left lane has priority
        benefit = acc_c_after - acc_c_before +
                  politeness_ * (acc_n_after - acc_n_before);
        VLOG(2) << " To left: " << acc_c_after << " - " << acc_c_before << " + "
                << politeness_ << " * (" << acc_n_after << " - " << acc_n_before
                << ") = " << benefit << " > "
                << acc_threshold + acceleration_bias_;
      } else {
        benefit = acc_c_after - acc_c_before +
                  politeness_ *
                      (acc_n_after - acc_n_before + acc_o_after - acc_o_before);
      }
      double threshold = acc_threshold;
      if (asymmetric_passing_rules_) {
        threshold += acceleration_bias_;
      }

      // Advantage criterion
      if (benefit > threshold) {
        if (asymmetric_passing_rules_) {
          // std::cout << "Go left. Ego improves by " << acc_c_after -
          // acc_c_before; std::cout << ", others improve by " << acc_n_after
          // - acc_n_before + acc_o_after - acc_o_before << std::endl;
        } else {
          VLOG(2) << "Go left. Ego improves from " << acc_c_before << " to"
                  << acc_c_after << ", others improve by "
                  << acc_n_after - acc_n_before;
        }
        acc_threshold = benefit;
        return std::make_pair(LaneChangeDecision::ChangeLeft, left_corridor);
      } else {
        VLOG(2) << " Benefit " << acc_c_after << " - " << acc_c_before << " + "
                << politeness_ << " * (" << acc_n_after << " - " << acc_n_before
                << ") = " << benefit << " would be less than the threshold "
                << threshold << ".";
      }
    } else {
      VLOG(2) << " To left would be unsafe: " << acc_n_after;
    }
  }
  return std::make_pair(LaneChangeDecision::KeepLane,
                        observed_world.GetLaneCorridor());
}
}  // namespace behavior
}  // namespace models
}  // namespace bark
