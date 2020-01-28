// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include "modules/models/behavior/mobil/mobil.hpp"
#include "modules/world/observed_world.hpp"
#include "modules/models/behavior/idm/idm_classic.hpp"
#include "modules/models/dynamic/integration.hpp"


namespace modules {
namespace models {
namespace behavior {

using world::ObservedWorld;
using world::Agent;
using world::objects::AgentPtr;
using dynamic::StateDefinition;
using world::map::LaneCorridorPtr;
using dynamic::State;

Trajectory BehaviorMobil::Plan(float delta_time, const ObservedWorld &observed_world) {
 /* if (!behavior_pure_pursuit_.is_following_line()) {
    // Initially set the followed line to the center line of the driving corridor
    behavior_pure_pursuit_.set_followed_line(observed_world.get_local_map()->get_driving_corridor().get_center());
  }

  // Determine whether to perform a lane change
  double acceleration;
  if (is_changing_lane_) {
    has_changed_lane_ = true;
    acceleration = 0;
    if (behavior_pure_pursuit_.has_reached_line()) {
      is_changing_lane_ = false;
    }
  } else {
    if (!has_changed_lane_) {
      InitiateLaneChangeIfBeneficial(observed_world);
    }
    auto driving_corridor = std::make_shared<DrivingCorridor>(observed_world.get_local_map()->get_driving_corridor());
    auto agent_in_front = observed_world.GetAgentInFront();
    acceleration = CalculateLongitudinalAcceleration(observed_world.get_ego_agent(), agent_in_front.first, agent_in_front.second.lon);
  }
*/
  float integration_time_delta = GetParams()->GetReal("integration_time_delta", "delta t for integration", 0.01);
  int n_trajectory_points = static_cast<int>(std::ceil(delta_time / integration_time_delta)) + 1;

  Trajectory traj(n_trajectory_points, static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  traj.row(0) = observed_world.CurrentEgoState();
/*
  for (int t = 1; t < n_trajectory_points; ++t) {
    double steering = behavior_pure_pursuit_.FindSteeringAngle(traj.row(t - 1));

    dynamic::Input input(2);
    input << acceleration, steering;

    traj.row(t) = dynamic::euler_int(*dynamic_model_, traj.row(t - 1), input, integration_time_delta);
  }

  set_last_trajectory(traj);*/
  return traj;
}

LaneChangeDecision BehaviorMobil::CheckIfLaneChangeBeneficial(const ObservedWorld &observed_world) {
  using world::map::Frenet;
  using modules::geometry::Point2d;
  
  std::shared_ptr<const Agent> ego_agent = observed_world.GetEgoAgent();

  const float vehicle_length = ego_agent->GetShape().front_dist_ + ego_agent->GetShape().rear_dist_;

  std::pair<AgentPtr, Frenet> follower_current_lane = observed_world.GetAgentBehind();
  std::pair<AgentPtr, Frenet> leader_current_lane = observed_world.GetAgentInFront();

  // acceleration of ego vehicle (before ... in the present situation)
  double acc_c_before = CalculateLongitudinalAccelerationTwoAgents(ego_agent, leader_current_lane.first, leader_current_lane.second.lon);

  // acceleration of old follower (before ... in the present situation)
  const double acc_o_before = CalculateLongitudinalAccelerationTwoAgents(follower_current_lane.first, ego_agent, follower_current_lane.second.lon);

  // acceleration of new follower (after a prospective lane change)
  // TODO(@Klemens): check distance_after term
  const double distane_after = follower_current_lane.second.lon + vehicle_length + leader_current_lane.second.lon;
  const double acc_o_after = CalculateLongitudinalAccelerationTwoAgents(follower_current_lane.first, leader_current_lane.first, distane_after);

  double acc_threshold = acceleration_threshold_;

  // TODO: can this be done more elegantly?
  const State ego_state = observed_world.CurrentEgoState();
  const float ego_x = ego_state(StateDefinition::X_POSITION);
  const float ego_y = ego_state(StateDefinition::Y_POSITION);

  auto road_corridor = observed_world.GetRoadCorridor();
  std::pair<LaneCorridorPtr, LaneCorridorPtr> left_right_lane_corridor = road_corridor->GetLeftRightLaneCorridor(Point2d(ego_x, ego_y));

  if (left_right_lane_corridor.second) {
    std::cout << " Right lane exists: " << std::endl;

    // TODO(@Klemens): Come up with a good interface to access those!!
    std::pair<AgentPtr, Frenet> right_following; // = observed_world.GetAgentBehind(right_corridor.first);
    std::pair<AgentPtr, Frenet> right_leading; // = observed_world.get_agent_in_front(right_corridor.first, true);

    // acceleration of ego vehicle (after a prospective lane change)
    double acc_c_after = CalculateLongitudinalAccelerationTwoAgents(ego_agent, right_leading.first, right_leading.second.lon);

    if (leader_current_lane.first && asymmetric_passing_rules_) {
      float ego_velocity = ego_state(StateDefinition::VEL_POSITION);
      float left_leading_velocity = leader_current_lane.first->GetCurrentState()(StateDefinition::VEL_POSITION);

      // Passing on the right is disallowed, therefore if the ego vehicle is faster than the leading vehicle on the left lane, its acceleration on the right
      // lane acc_c_after cannot be higher than its acceleration on the left lane acc_c_before
      if (ego_velocity > left_leading_velocity && left_leading_velocity > critical_velocity_) {
        acc_c_after = std::min(acc_c_before, acc_c_after);
      }
    }

    const double distance_before = right_following.second.lon + vehicle_length + right_leading.second.lon;
    // acceleration of new follower (before ... in the present situation)
    const double acc_n_before = CalculateLongitudinalAccelerationTwoAgents(right_following.first, right_leading.first, distance_before);
    // acceleration of new follower (after a prospective lane change)
    const double acc_n_after = CalculateLongitudinalAccelerationTwoAgents(right_following.first, ego_agent, right_following.second.lon);

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
        //behavior_pure_pursuit_.set_followed_line(right_corridor.first->get_center());
        is_changing_lane_ = true;
        return LaneChangeDecision::ChangeRight;
      }
    } else {
      // std::cout << " To right would be unsafe: " << acc_n_after << " >= " << -safe_decel_ << std::endl;
    }
  }

  // Try change to left lane
  if (left_right_lane_corridor.first) {
     std::cout << " Left Lane exists " << std::endl;

    // TODO(@Klemens): fill
    std::pair<AgentPtr, Frenet> left_following; // = observed_world.get_agent_behind(left_corridor.first);
    std::pair<AgentPtr, Frenet> left_leading; // = observed_world.get_agent_in_front(left_corridor.first, true);

    if (left_following.first == nullptr) {
      // std::cout << " Agent " << observed_world.get_ego_agent()->get_agent_id() << ": Could go left, nobody there" << std::endl;
    }

    const double acc_c_after = CalculateLongitudinalAccelerationTwoAgents(ego_agent, left_leading.first, left_leading.second.lon);

    if (left_leading.first && asymmetric_passing_rules_) {

      float ego_velocity = ego_state(StateDefinition::VEL_POSITION);
      float left_leading_velocity = left_leading.first->GetCurrentState()(StateDefinition::VEL_POSITION);

      // Passing on the right is disallowed, therefore if the ego vehicle is faster than the leading vehicle on the left lane, its acceleration on the right
      // lane acc_c_before cannot be higher than its acceleration on the right lane acc_c_after
      if (ego_velocity > left_leading_velocity && left_leading_velocity > critical_velocity_) {
        acc_c_before = std::min(acc_c_before, acc_c_after);
      }
    }

    // acceleration of new follower (before ... in the present situation)
    const double acc_n_before = CalculateLongitudinalAccelerationTwoAgents(left_following.first, left_leading.first, left_following.second.lon + vehicle_length + left_leading.second.lon);
    // acceleration of new follower (after a prospective lane change)
    const double acc_n_after = CalculateLongitudinalAccelerationTwoAgents(left_following.first, ego_agent, left_following.second.lon);

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
        // behavior_pure_pursuit_.set_followed_line(left_corridor.first->get_center());
        is_changing_lane_ = true;
        return LaneChangeDecision::ChangeLeft;
      } else {
        // std::cout << " Benefit " << acc_c_after << " - " << acc_c_before << " + " << politeness_ << " * (" << acc_n_after << " - " << acc_n_before << ") = " << benefit
        //           << " would be less than the threshold " << threshold << "." << std::endl;
      }
    } else {
      // std::cout << " To left would be unsafe: " << acc_n_after << std::endl;
    }
  }
  return LaneChangeDecision::KeepLane;
  
}


} // namespace behavior
} // namespace models
} // namespace modules