// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include "modules/models/behavior/mobil/mobil.hpp"
#include "modules/models/behavior/idm/idm_classic.hpp"
#include "modules/models/dynamic/integration.hpp"
#include "modules/models/dynamic/single_track.hpp"



namespace modules {
namespace models {
namespace behavior {

using world::ObservedWorld;
using world::Agent;
using world::objects::AgentPtr;
using dynamic::StateDefinition;
using world::map::LaneCorridorPtr;
using dynamic::State;
using modules::geometry::Line;
using modules::commons::transformation::FrenetPosition;
using modules::models::dynamic::DynamicModelPtr;
using modules::models::dynamic::CalculateSteeringAngle;

Trajectory BehaviorMobil::Plan(float delta_time, const ObservedWorld &observed_world) {

  const DynamicModelPtr dynamic_model = observed_world.GetEgoAgent()->GetDynamicModel();
  auto single_track = std::dynamic_pointer_cast<dynamic::SingleTrackModel>(dynamic_model);
  if (!single_track) {
    LOG(FATAL) << "Only SingleTrack as dynamic model supported!";
  }

  //! Determine whether to perform a lane change
  if (mobil_state_ == MobilState::IsChanging) {
    //! checks if lane change has been finished ...
    FrenetPosition f_state = FrenetPosition(observed_world.CurrentEgoPosition(), target_corridor_->GetCenterLine());
    if (abs(f_state.lat) < 0.01) {
      //! if very close to target line, lane change is finished ...
      // TODO(@Klemens): Make this more generic, maybe based on integation step size?
      mobil_state_ = MobilState::Idle;
    }
  } else {
    LaneChangeDecision decision;
    auto ben  = CheckIfLaneChangeBeneficial(observed_world);
    decision = ben.first;
    target_corridor_ = ben.second;
    std::cout << "Decision " << decision << std::endl;
  }

  const int num_traj_time_points = 11;
  dynamic::Trajectory traj(num_traj_time_points,
                           int(StateDefinition::MIN_STATE_SIZE));
  float const dt = delta_time / (num_traj_time_points - 1);

  traj.row(0) = observed_world.CurrentEgoState();

  double acc;
  if (mobil_state_ == MobilState::Idle) {
    auto agent_in_front = observed_world.GetAgentInFront();
    acc = CalcLongAccTwoAgents(observed_world.GetEgoAgent(), agent_in_front.first, agent_in_front.second.lon);
  } else {
    acc = 0;
  }

  for (int i = 1; i < num_traj_time_points; ++i) {
    double angle = CalculateSteeringAngle(single_track, traj.row(i - 1), target_corridor_->GetCenterLine(), crosstrack_error_gain_);

    dynamic::Input input(2);
    input << acc, angle;
    traj.row(i) = dynamic::euler_int(*dynamic_model, traj.row(i - 1), input, dt);
  }

  SetLastTrajectory(traj);
  return traj;
}

std::pair<LaneChangeDecision, LaneCorridorPtr> BehaviorMobil::CheckIfLaneChangeBeneficial(const ObservedWorld &observed_world) {
  using modules::geometry::Point2d;
  using world::FrontRearAgents;
  using StateDefinition::VEL_POSITION;
  
  std::shared_ptr<const Agent> ego_agent = observed_world.GetEgoAgent();

  const float vehicle_length = ego_agent->GetShape().front_dist_ + ego_agent->GetShape().rear_dist_;

  FrontRearAgents agents_current_lane = observed_world.GetAgentFrontRear();
  std::pair<AgentPtr, FrenetPosition> follower_current = agents_current_lane.rear;
  std::pair<AgentPtr, FrenetPosition> leader_current = agents_current_lane.front;

  // acceleration of ego vehicle (before ... in the present situation)
  double acc_c_before = CalcLongAccTwoAgents(ego_agent, leader_current.first, leader_current.second.lon);

  // acceleration of old follower (before ... in the present situation)
  double acc_o_before, acc_o_after;
  if (follower_current.first) {  // no follower
    acc_o_before = CalcLongAccTwoAgents(follower_current.first, ego_agent, follower_current.second.lon);
    const double distance_after = follower_current.second.lon + vehicle_length + leader_current.second.lon;
    acc_o_after = CalcLongAccTwoAgents(follower_current.first, leader_current.first, distance_after);
  } 
  else {
    acc_o_before = 0, acc_o_after = 0;
  }

  // acceleration of new follower (after a prospective lane change)
  // TODO(@Klemens): check distance_after term

  double acc_threshold = acceleration_threshold_;
  const Point2d ego_pos = observed_world.CurrentEgoPosition();

  auto road_corridor = observed_world.GetRoadCorridor();
  LaneCorridorPtr left_corridor;
  LaneCorridorPtr right_corridor;
  std::tie(left_corridor, right_corridor) = road_corridor->GetLeftRightLaneCorridor(ego_pos);

  AgentId ego_agent_id = observed_world.GetEgoAgentId();

  if (right_corridor) {
    //! Right lane exists
    FrontRearAgents agents_right_lane = observed_world.GetAgentFrontRearForId(ego_agent_id, right_corridor);
    std::pair<AgentPtr, FrenetPosition> follower_right = agents_right_lane.rear;
    std::pair<AgentPtr, FrenetPosition> leader_right = agents_right_lane.front;

    // acceleration of ego vehicle (after a prospective lane change)
    double acc_c_after = CalcLongAccTwoAgents(ego_agent, leader_right.first, leader_right.second.lon);

    if (leader_current.first && asymmetric_passing_rules_) {
      float vel_ego = observed_world.CurrentEgoState()(VEL_POSITION);
      float vel_leader_left = leader_current.first->GetCurrentState()(VEL_POSITION);

      // Passing on the right is disallowed, therefore if the ego vehicle is faster than the leading vehicle on the left lane, its acceleration on the right
      // lane acc_c_after cannot be higher than its acceleration on the left lane acc_c_before
      if (vel_ego > vel_leader_left && vel_leader_left > critical_velocity_) {
        acc_c_after = std::min(acc_c_before, acc_c_after);
      }
    }
    
    double acc_n_before, acc_n_after;
    if (follower_right.first) {
      const double distance_before = follower_right.second.lon + vehicle_length + leader_right.second.lon;
      // acceleration of new follower (before ... in the present situation)
      acc_n_before = CalcLongAccTwoAgents(follower_right.first, leader_right.first, distance_before);
      // acceleration of new follower (after a prospective lane change)
      acc_n_after = CalcLongAccTwoAgents(follower_right.first, ego_agent, follower_right.second.lon);
    }
    else {
      acc_n_before = 0, acc_n_after = 0;
    }

    // Safety criterion ensures that the after lane change, the new follower can still safely decelerate to avoid a crash
    if (acc_n_after >= -safe_deceleration_) {
      double benefit;
      if (asymmetric_passing_rules_) {
        // Neglect the disadvantage of the following vehicle on the right lane (acc_n_after - acc_n_before), because the left lane has priority
        benefit = acc_c_after - acc_c_before + politeness_ * (acc_o_after - acc_o_before);
        // std::cout << " to right: " << acc_c_after << " - " << acc_c_before << " + " << politeness_ << " * (" << acc_o_after << " - " << acc_o_before << ")";
        // std::cout << " = " << benefit << " > " << acc_threshold - acceleration_bias_ << std::endl;
      } else {
        benefit = acc_c_after - acc_c_before + politeness_ * (acc_n_after - acc_n_before + acc_o_after - acc_o_before);
      }
      double threshold = acc_threshold;
      if (asymmetric_passing_rules_) {
        threshold -= acceleration_bias_;
      }

      // Advantage criterion
      if (benefit > threshold) {
        if (asymmetric_passing_rules_) {
          // std::cout << "Go right. Ego improves by " << acc_c_after - acc_c_before;
          // std::cout << ", others improve by " << acc_n_after - acc_n_before + acc_o_after - acc_o_before << std::endl;
        } else {
          // std::cout << "Go right. Ego improves by " << acc_c_after - acc_c_before;
          // std::cout << ", others improve by " << acc_o_after - acc_o_before << std::endl;
        }
        acc_threshold = benefit;
        return std::make_pair(LaneChangeDecision::ChangeRight, right_corridor);
      }
    } else {
      // std::cout << " To right would be unsafe: " << acc_n_after << " >= " << -safe_decel_ << std::endl;
    }
  }

  if (left_corridor) {
     //! Left Lane exists

    FrontRearAgents agents_left_lane = observed_world.GetAgentFrontRearForId(ego_agent_id, left_corridor);
    std::pair<AgentPtr, FrenetPosition> follower_left = agents_left_lane.rear;
    std::pair<AgentPtr, FrenetPosition> leader_left = agents_left_lane.front;

    // acceleration of ego vehicle (after a prospective lane change)
    const double acc_c_after = CalcLongAccTwoAgents(ego_agent, leader_left.first, leader_left.second.lon);

    if (leader_left.first && asymmetric_passing_rules_) {
      float vel_ego = observed_world.CurrentEgoState()(VEL_POSITION);
      float vel_leader_left = leader_left.first->GetCurrentState()(VEL_POSITION);

      // Passing on the right is disallowed, therefore if the ego vehicle is faster than the leading vehicle on the left lane, its acceleration on the right
      // lane acc_c_before cannot be higher than its acceleration on the right lane acc_c_after
      if (vel_ego > vel_leader_left && vel_leader_left > critical_velocity_) {
        acc_c_before = std::min(acc_c_before, acc_c_after);
      }
    }

    double acc_n_before, acc_n_after;
    if (follower_left.first) {
      const double distance_before = follower_left.second.lon + vehicle_length + leader_left.second.lon;
      // acceleration of new follower (before ... in the present situation)
      acc_n_before = CalcLongAccTwoAgents(follower_left.first, leader_left.first, distance_before);
      // acceleration of new follower (after a prospective lane change)
      acc_n_after = CalcLongAccTwoAgents(follower_left.first, ego_agent, follower_left.second.lon);
    }
    else {
      acc_n_before = 0, acc_n_after = 0;
    }

    // Safety criterion ensures that the after lane change, the new follower can still safely decelerate to avoid a crash
    if (acc_n_after >= -safe_deceleration_) {
      double benefit;
      if (asymmetric_passing_rules_) {
        // Neglect the disadvantage of the following vehicle on the right lane (acc_o_after - acc_o_before), because the left lane has priority
        benefit = acc_c_after - acc_c_before + politeness_ * (acc_n_after - acc_n_before);
        // std::cout << " To left: " << acc_c_after << " - " << acc_c_before << " + " << politeness_ << " * (" << acc_n_after << " - " << acc_n_before << ")";
        // std::cout << " = " << benefit << " > " << acc_threshold + acceleration_bias_ << std::endl;
      } else {
        benefit = acc_c_after - acc_c_before + politeness_ * (acc_n_after - acc_n_before + acc_o_after - acc_o_before);
      }
      double threshold = acc_threshold;
      if (asymmetric_passing_rules_) {
        threshold += acceleration_bias_;
      }

      // Advantage criterion
      if (benefit > threshold) {
        if (asymmetric_passing_rules_) {
          // std::cout << "Go left. Ego improves by " << acc_c_after - acc_c_before;
          // std::cout << ", others improve by " << acc_n_after - acc_n_before + acc_o_after - acc_o_before << std::endl;
        } else {
          // std::cout << "Go left. Ego improves from " << acc_c_before << " to " << acc_c_after;
          // std::cout << ", others improve by " << acc_n_after - acc_n_before << std::endl;
        }
        acc_threshold = benefit;
        return std::make_pair(LaneChangeDecision::ChangeLeft, left_corridor);
      } else {
        // std::cout << " Benefit " << acc_c_after << " - " << acc_c_before << " + " << politeness_ << " * (" << acc_n_after << " - " << acc_n_before << ") = " << benefit
        //           << " would be less than the threshold " << threshold << "." << std::endl;
      }
    } else {
      // std::cout << " To left would be unsafe: " << acc_n_after << std::endl;
    }
  }
  return std::make_pair(LaneChangeDecision::KeepLane, observed_world.GetLaneCorridor());
}


} // namespace behavior
} // namespace models
} // namespace modules