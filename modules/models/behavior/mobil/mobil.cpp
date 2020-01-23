#include "modules/models/behavior/mobil/mobil.hpp"
#include "modules/world/observed_world.hpp"
#include "modules/models/behavior/idm/idm_classic.hpp"
#include "modules/models/dynamic/integration.hpp"


namespace modules {
namespace models {
namespace behavior {

using world::ObservedWorld;
using world::objects::AgentPtr;
using world::map::DrivingCorridor;
using world::map::DrivingCorridorPtr;
using dynamic::StateDefinition;

using ConstAgentPtr = std::shared_ptr<const world::objects::Agent>;

Trajectory BehaviorMobil::Plan(float delta_time, const ObservedWorld &observed_world) {
  if (!behavior_pure_pursuit_.is_following_line()) {
    // Initially set the followed line to the center line of the driving corridor
    behavior_pure_pursuit_.set_followed_line(observed_world.get_local_map()->get_driving_corridor().get_center());
  }

  // Determine whether to perform a lane change
  double acceleration;
  if (is_changing_lane_) {
    acceleration = 0;
    if (behavior_pure_pursuit_.has_reached_line()) {
      is_changing_lane_ = false;
    }
  } else {
    InitiateLaneChangeIfBeneficial(observed_world);
    auto driving_corridor = std::make_shared<DrivingCorridor>(observed_world.get_local_map()->get_driving_corridor());
    auto agent_in_front = observed_world.get_agent_in_front(driving_corridor, true);
    acceleration = CalculateLongitudinalAcceleration(observed_world.get_ego_agent(), agent_in_front.first, agent_in_front.second.lon);
  }

  float integration_time_delta = get_params()->get_real("integration_time_delta", "delta t for integration", 0.01);
  int n_trajectory_points = static_cast<int>(std::ceil(delta_time / integration_time_delta)) + 1;

  Trajectory traj(n_trajectory_points, static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  traj.row(0) = observed_world.current_ego_state();

  for (int t = 1; t < n_trajectory_points; ++t) {
    double steering = behavior_pure_pursuit_.FindSteeringAngle(traj.row(t - 1));

    dynamic::Input input(2);
    input << acceleration, steering;

    traj.row(t) = dynamic::euler_int(*dynamic_model_, traj.row(t - 1), input, integration_time_delta);
  }

  set_last_trajectory(traj);
  return traj;
}

void BehaviorMobil::InitiateLaneChangeIfBeneficial(const ObservedWorld &observed_world) {
  using world::map::Frenet;
  DrivingCorridorPtr current_corridor = std::make_shared<DrivingCorridor>(observed_world.get_local_map()->get_driving_corridor());
  // std::cout << " Current: " << observed_world.get_map()->get_lane(current_corridor->get_lane_ids().front().second)->get_lane_position() << std::endl;

  ConstAgentPtr ego_agent = observed_world.get_ego_agent();
  const float ego_length = ego_agent->get_shape().rear_dist_ + ego_agent->get_shape().front_dist_;

  std::pair<AgentPtr, Frenet> current_following = observed_world.get_agent_behind(current_corridor);
  std::pair<AgentPtr, Frenet> current_leading = observed_world.get_agent_in_front(current_corridor, true);

  double acc_c_before = CalculateLongitudinalAcceleration(ego_agent, current_leading.first, current_leading.second.lon);

  const double acc_o_before = CalculateLongitudinalAcceleration(current_following.first, ego_agent, current_following.second.lon);
  const double acc_o_after = CalculateLongitudinalAcceleration(current_following.first, current_leading.first, current_following.second.lon + ego_length + current_leading.second.lon);

  double acc_threshold = acceleration_threshold_;

  // Try change to right lane
  std::pair<DrivingCorridorPtr, bool> right_corridor = ego_agent->get_local_map()->get_right_adjacent_corridor(current_corridor);
  if (right_corridor.second) {
    // std::cout << " Right: " << observed_world.get_map()->get_lane(right_corridor.first->get_lane_ids().front().second)->get_lane_position() << std::endl;

    std::pair<AgentPtr, Frenet> right_following = observed_world.get_agent_behind(right_corridor.first);
    std::pair<AgentPtr, Frenet> right_leading = observed_world.get_agent_in_front(right_corridor.first, true);

    double acc_c_after = CalculateLongitudinalAcceleration(ego_agent, right_leading.first, right_leading.second.lon);

    if (asymmetric_passing_rules_ && current_leading.first != nullptr) {
      // Passing on the right is disallowed, therefore if the ego vehicle is faster than the leading vehicle on the left lane, its acceleration on the right
      // lane acc_c_after cannot be higher than its acceleration on the left lane acc_c_before

      float ego_velocity = ego_agent->get_current_state()(StateDefinition::VEL_POSITION);
      float left_leading_velocity = current_leading.first->get_current_state()(StateDefinition::VEL_POSITION);

      if (ego_velocity > left_leading_velocity && left_leading_velocity > critical_velocity_) {
        acc_c_after = std::min(acc_c_before, acc_c_after);
      }
    }

    const double acc_n_before = CalculateLongitudinalAcceleration(right_following.first, right_leading.first, right_following.second.lon + ego_length + right_leading.second.lon);
    const double acc_n_after = CalculateLongitudinalAcceleration(right_following.first, ego_agent, right_following.second.lon);

    if (acc_n_after >= -safe_decel_) {                                                                                      // Safety criterion
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
      if (benefit > threshold) {                                                                                            // Advantage criterion
        if (asymmetric_passing_rules_) {
          // std::cout << "Go right. Ego improves by " << acc_c_after - acc_c_before;
          // std::cout << ", others improve by " << acc_n_after - acc_n_before + acc_o_after - acc_o_before << std::endl;
        } else {
          // std::cout << "Go right. Ego improves by " << acc_c_after - acc_c_before;
          // std::cout << ", others improve by " << acc_o_after - acc_o_before << std::endl;
        }
        acc_threshold = benefit;
        behavior_pure_pursuit_.set_followed_line(right_corridor.first->get_center());
        is_changing_lane_ = true;
      }
    } else {
      // std::cout << " To right would be unsafe: " << acc_n_after << " >= " << -safe_decel_ << std::endl;
    }
  }

  // Try change to left lane
  std::pair<DrivingCorridorPtr, bool> left_corridor = ego_agent->get_local_map()->get_left_adjacent_corridor(current_corridor);
  if (left_corridor.second) {
    // std::cout << " Left: " << observed_world.get_map()->get_lane(left_corridor.first->get_lane_ids().front().second)->get_lane_position() << std::endl;

    std::pair<AgentPtr, Frenet> left_following = observed_world.get_agent_behind(left_corridor.first);
    std::pair<AgentPtr, Frenet> left_leading = observed_world.get_agent_in_front(left_corridor.first, true);

    if (left_following.first == nullptr) {
      // std::cout << " Agent " << observed_world.get_ego_agent()->get_agent_id() << ": Could go left, nobody there" << std::endl;
    }

    const double acc_c_after = CalculateLongitudinalAcceleration(ego_agent, left_leading.first, left_leading.second.lon);

    if (asymmetric_passing_rules_ && left_leading.first != nullptr) {
      // Passing on the right is disallowed, therefore if the ego vehicle is faster than the leading vehicle on the left lane, its acceleration on the right
      // lane acc_c_before cannot be higher than its acceleration on the right lane acc_c_after

      float ego_velocity = ego_agent->get_current_state()(StateDefinition::VEL_POSITION);
      float left_leading_velocity = left_leading.first->get_current_state()(StateDefinition::VEL_POSITION);

      if (ego_velocity > left_leading_velocity && left_leading_velocity > critical_velocity_) {
        acc_c_before = std::min(acc_c_before, acc_c_after);
      }
    }

    const double acc_n_before = CalculateLongitudinalAcceleration(left_following.first, left_leading.first, left_following.second.lon + ego_length + left_leading.second.lon);
    const double acc_n_after = CalculateLongitudinalAcceleration(left_following.first, ego_agent, left_following.second.lon);

    if (acc_n_after >= -safe_decel_) {                                                                                          // Safety criterion
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
      if (benefit > threshold) {                                                                                            // Advantage criterion
        if (asymmetric_passing_rules_) {
          // std::cout << "Go left. Ego improves by " << acc_c_after - acc_c_before;
          // std::cout << ", others improve by " << acc_n_after - acc_n_before + acc_o_after - acc_o_before << std::endl;
        } else {
          // std::cout << "Go left. Ego improves from " << acc_c_before << " to " << acc_c_after;
          // std::cout << ", others improve by " << acc_n_after - acc_n_before << std::endl;
        }
        acc_threshold = benefit;
        behavior_pure_pursuit_.set_followed_line(left_corridor.first->get_center());
        is_changing_lane_ = true;
      } else {
        // std::cout << " Benefit " << acc_c_after << " - " << acc_c_before << " + " << politeness_ << " * (" << acc_n_after << " - " << acc_n_before << ") = " << benefit
        //           << " would be less than the threshold " << threshold << "." << std::endl;
      }
    } else {
      // std::cout << " To left would be unsafe: " << acc_n_after << std::endl;
    }
  }
}

double BehaviorMobil::CalculateLongitudinalAcceleration(const ConstAgentPtr &ego_agent, const ConstAgentPtr &leading_vehicle, const double distance) const {
  if (ego_agent == nullptr) {
    return 0.0;
  }

  // Obtain IDM parameters of the ego agent
  // TODO(@AKreutz): Does this make sense for predicting the behavior of others? Maybe it makes more sense to use current velocity for desired velocity,
  // and default values for everything else
  float desired_velocity;
  float minimum_spacing;
  float desired_time_headway;
  float max_acceleration;
  float comfortable_braking_acceleration;
  int exponent;

  std::shared_ptr<BehaviorIDMClassic> behavior_idm;

  std::shared_ptr<BehaviorMobil> behavior_mobil = std::dynamic_pointer_cast<BehaviorMobil>(ego_agent->get_behavior_model());
  if (behavior_mobil) {
    // If agent behavior is a MOBIL model, obtain its longitudinal_behavior_ model and cast it to IDM
    behavior_idm = std::dynamic_pointer_cast<BehaviorIDMClassic>(behavior_mobil->longitudinal_behavior_);
  } else {
    // Cast the agent's behavior model to IDM
    behavior_idm = std::dynamic_pointer_cast<BehaviorIDMClassic>(ego_agent->get_behavior_model());
  }

  if (behavior_idm) {
    desired_velocity = behavior_idm->get_desired_velocity();
    minimum_spacing = behavior_idm->get_minimum_spacing();
    desired_time_headway = behavior_idm->get_desired_time_headway();
    max_acceleration = behavior_idm->get_max_acceleration();
    comfortable_braking_acceleration = behavior_idm->get_comfortable_braking_acceleration();
    exponent = behavior_idm->get_exponent();
  } else {
    // std::cout << "Using default" << std::endl;
    // default parameters
    // desired_velocity = 15.0f;
    desired_velocity = ego_agent->get_current_state()(StateDefinition::VEL_POSITION);
    minimum_spacing = 2.0f;
    desired_time_headway = 1.5f;
    max_acceleration = 1.7f;
    comfortable_braking_acceleration = 1.67f;
    exponent = 4;
  }

  // relative velocity and longitudinal distance
  const float ego_velocity = ego_agent->get_current_state()(StateDefinition::VEL_POSITION);
  double interaction_term = 0.0f;
  if(leading_vehicle) {
    // Leading vehicle exists in driving corridor, we calculate interaction term

    const float lead_velocity = leading_vehicle->get_current_state()(StateDefinition::VEL_POSITION);

    const float vehicle_length = ego_agent->get_shape().front_dist_ +
                                  leading_vehicle->get_shape().rear_dist_;
    const double net_distance = distance - vehicle_length;
    if (net_distance < 0) {
      // Ego agent and leading vehicle are colliding
      return -std::numeric_limits<double>::max();
    }
    
    const double net_velocity = ego_velocity - lead_velocity;

    const double helper_state = minimum_spacing + ego_velocity*desired_time_headway +
                      (ego_velocity*net_velocity) / (2*sqrt(max_acceleration*comfortable_braking_acceleration));

    BARK_EXPECT_TRUE(!std::isnan(helper_state));
    interaction_term = (helper_state / net_distance)*(helper_state / net_distance);
    BARK_EXPECT_TRUE(!std::isnan(interaction_term));
  }

  // Calculate acceleration based on https://en.wikipedia.org/wiki/Intelligent_driver_model (Maybe look into paper for other implementation aspects)
  return max_acceleration * ( 1 - pow(ego_velocity / desired_velocity, exponent) - interaction_term);
}

} // namespace behavior
} // namespace models
} // namespace modules
