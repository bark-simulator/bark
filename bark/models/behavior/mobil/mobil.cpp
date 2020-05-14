// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/models/behavior/mobil/mobil.hpp"
#include <algorithm>
#include <memory>
#include <utility>
#include "bark/models/behavior/idm/idm_classic.hpp"
#include "bark/models/dynamic/integration.hpp"
#include "bark/models/dynamic/single_track.hpp"

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
using StateDefinition::VEL_POSITION;
using world::Agent;
using world::AgentFrenetPair;
using world::AgentId;
using world::ObservedWorld;
using world::map::LaneCorridorPtr;
using world::objects::AgentPtr;

Trajectory BehaviorMobil::Plan(float delta_time,
                               const ObservedWorld& observed_world) {
  SetBehaviorStatus(BehaviorStatus::VALID);

  const DynamicModelPtr dynamic_model =
      observed_world.GetEgoAgent()->GetDynamicModel();
  auto single_track =
      std::dynamic_pointer_cast<dynamic::SingleTrackModel>(dynamic_model);
  if (!single_track) {
    LOG(FATAL) << "Only SingleTrack as dynamic model supported!";
  }

  VLOG(2) << "Agent: " << observed_world.GetEgoAgentId() << ", Mobil State "
          << mobil_state_;
  //! Determine whether to perform a lane change
  if (mobil_state_ == MobilState::IsChanging) {
    //! checks if lane change has been finished ...
    FrenetPosition f_state = FrenetPosition(observed_world.CurrentEgoPosition(),
                                            target_corridor_->GetCenterLine());
    if (abs(f_state.lat) < 0.01) {
      //! if very close to target line, lane change is finished ...
      // TODO(@Klemens): Make this more generic, maybe based on integation step
      // size?
      mobil_state_ = MobilState::Idle;
    }
  } else {
    LaneChangeDecision decision;
    std::tie(decision, target_corridor_) =
        CheckIfLaneChangeBeneficial(observed_world);
    VLOG(2) << "Decision " << decision;
    if (decision != LaneChangeDecision::KeepLane)
      mobil_state_ = MobilState::IsChanging;
  }

  const int num_traj_time_points = 11;
  dynamic::Trajectory traj(num_traj_time_points,
                           static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  float const dt = delta_time / (num_traj_time_points - 1);

  traj.row(0) = observed_world.CurrentEgoState();

  // Get leading agent in target lane (lane we are changing to in case of a lane
  // change, otherwise it's the lane the agent is currently in)
  AgentId id_ego = observed_world.GetEgoAgentId();
  AgentFrenetPair agent_in_front =
      observed_world.GetAgentFrontRearForId(id_ego, target_corridor_).front;

  std::shared_ptr<const Agent> ego_agent = observed_world.GetEgoAgent();
  FrenetPosition frenet_ego = ego_agent->CurrentFrenetPosition();

  double net_distance = .0f;
  double vel_other = 1e6;
  if (agent_in_front.first) {
    net_distance =
        CalcNetDistanceFromFrenet(ego_agent, frenet_ego, agent_in_front.first,
                                  agent_in_front.second + frenet_ego);
    dynamic::State other_vehicle_state =
        agent_in_front.first->GetCurrentState();
    vel_other = other_vehicle_state(StateDefinition::VEL_POSITION);
  }

  float vel_i;
  double acc;
  double traveled_ego;
  double traveled_other;
  for (int i = 1; i < num_traj_time_points; ++i) {
    vel_i = traj(i - 1, StateDefinition::VEL_POSITION);

    if (agent_in_front.first) {
      acc = idm_->CalcIDMAcc(net_distance, vel_i, vel_other);
      traveled_ego = +0.5f * acc * dt * dt + vel_i * dt;
      traveled_other = vel_other * dt;
      net_distance += traveled_other - traveled_ego;
    } else {
      Point2d pos_i(traj(i - 1, StateDefinition::X_POSITION),
                    traj(i - 1, StateDefinition::Y_POSITION));
      acc = CalcLongRawAccWithoutLeader(target_corridor_, pos_i, vel_i);
      const double acc_max = idm_->GetMaxAcceleration();
      acc = std::max(std::min(acc, acc_max), -acc_max);
    }

    double angle = CalculateSteeringAngle(single_track, traj.row(i - 1),
                                          target_corridor_->GetCenterLine(),
                                          crosstrack_error_gain_);

    dynamic::Input input(2);
    input << acc, angle;
    traj.row(i) =
        dynamic::euler_int(*dynamic_model, traj.row(i - 1), input, dt);
    // Do not allow negative speeds
    traj(i, StateDefinition::VEL_POSITION) =
        std::max(traj(i, StateDefinition::VEL_POSITION), 0.0f);
  }

  SetLastTrajectory(traj);
  return traj;
}

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
    const modules::geometry::Point2d& pos, const float vel) {
  double acc;
  if (stop_at_lane_ending_) {
    // Brake at end of lane corridor
    // const double net_distance =
    // lane_corridor->LengthUntilEnd(agent->GetCurrentPosition()) -
    // agent->GetShape().front_dist_;
    const double net_distance = lane_corridor->LengthUntilEnd(pos) - 15.0;
    // setting vel_other to zero
    acc = idm_->CalcRawIDMAcc(net_distance, vel, 0.0);
  } else {
    acc = idm_->GetMaxAcceleration() * idm_->CalcFreeRoadTerm(vel);
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
    acc_c_before = idm_->CalcRawIDMAcc(
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
    acc_o_before = idm_->CalcRawIDMAcc(
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
      acc_o_after = idm_->CalcRawIDMAcc(
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
      acc_c_after = idm_->CalcRawIDMAcc(
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
        acc_n_before = idm_->CalcRawIDMAcc(
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
          idm_->CalcRawIDMAcc(dist_n_after, vel_follower_right, vel_ego);
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
      acc_c_after = idm_->CalcRawIDMAcc(
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
        acc_n_before = idm_->CalcRawIDMAcc(
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
          idm_->CalcRawIDMAcc(dist_n_after, vel_follower_left, vel_ego);
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
}  // namespace modules
