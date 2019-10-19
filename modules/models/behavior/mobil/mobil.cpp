#include "modules/models/behavior/mobil/mobil.hpp"
#include "modules/world/observed_world.hpp"
#include "modules/models/behavior/idm/idm_classic.hpp"


namespace modules {
namespace models {
namespace behavior {

using world::ObservedWorld;
using world::objects::AgentPtr;
using dynamic::StateDefinition;

using ConstAgentPtr = std::shared_ptr<const world::objects::Agent>;

Trajectory BehaviorMobil::Plan(float delta_time, const ObservedWorld &observed_world) {
  if (!behavior_pure_pursuit_.is_following_line()) {
    // Initially set the followed line to the center line of the driving corridor
    behavior_pure_pursuit_.set_followed_line(observed_world.get_ego_agent()->get_local_map()->get_driving_corridor().get_center());
  }
  if (current_corridor_ == nullptr) {
    current_corridor_ = std::make_shared<world::map::DrivingCorridor>(observed_world.get_ego_agent()->get_local_map()->get_driving_corridor());
  }

  // Determine whether to perform a lane change
  double acceleration;
  if (is_changing_lane_) {
    ConcludeLaneChange(observed_world);
    acceleration = 0;
  } else {
    InitiateLaneChangeIfBeneficial(observed_world);
    auto agent_in_front = observed_world.get_agent_in_front(current_corridor_);
    acceleration = CalculateLongitudinalAcceleration(observed_world.get_ego_agent(), agent_in_front.first, agent_in_front.second);
  }

  double steering = behavior_pure_pursuit_.FindSteeringAngle(observed_world.current_ego_state());

  Input input(2);
  input << acceleration, steering;
  dynamic_behavior_model_.set_action(input);

  Trajectory trajectory = dynamic_behavior_model_.Plan(delta_time, observed_world);
  set_last_trajectory(trajectory);

  return trajectory;
}

void BehaviorMobil::InitiateLaneChangeIfBeneficial(const ObservedWorld &observed_world) {
  std::cout << " Current: " << observed_world.get_map()->get_lane(current_corridor_->get_lane_ids().front().second)->get_lane_position() << std::endl;

  ConstAgentPtr ego_agent = observed_world.get_ego_agent();
  const float ego_length = ego_agent->get_shape().rear_dist_ + ego_agent->get_shape().front_dist_;

  std::pair<AgentPtr, double> current_following = observed_world.get_agent_behind(current_corridor_);
  std::pair<AgentPtr, double> current_leading = observed_world.get_agent_in_front(current_corridor_);

  double acc_c_before = CalculateLongitudinalAcceleration(ego_agent, current_leading.first, current_leading.second);

  const double acc_o_before = CalculateLongitudinalAcceleration(current_following.first, ego_agent, current_following.second);
  const double acc_o_after = CalculateLongitudinalAcceleration(current_following.first, current_leading.first, current_following.second + ego_length + current_leading.second);

  double acc_threshold = acceleration_threshold_;

  // Try change to right lane
  std::pair<world::map::DrivingCorridorPtr, bool> right_corridor = ego_agent->get_local_map()->get_right_adjacent_corridor(current_corridor_);
  if (right_corridor.second) {
    // std::cout << " Right: " << observed_world.get_map()->get_lane(right_corridor.first->get_lane_ids().front().second)->get_lane_position() << std::endl;

    std::pair<AgentPtr, double> right_following = observed_world.get_agent_behind(right_corridor.first);
    std::pair<AgentPtr, double> right_leading = observed_world.get_agent_in_front(right_corridor.first);

    double acc_c_after = CalculateLongitudinalAcceleration(ego_agent, right_leading.first, right_leading.second);

    if (asymmetric_passing_rules_ && current_leading.first != nullptr) {
      // Passing on the right is disallowed, therefore if the ego vehicle is faster than the leading vehicle on the left lane, its acceleration on the right
      // lane acc_c_after cannot be higher than its acceleration on the left lane acc_c_before

      float ego_velocity = ego_agent->get_current_state()(StateDefinition::VEL_POSITION);
      float left_leading_velocity = current_leading.first->get_current_state()(StateDefinition::VEL_POSITION);

      if (ego_velocity > left_leading_velocity && left_leading_velocity > critical_velocity_) {
        acc_c_after = std::min(acc_c_before, acc_c_after);
      }
    }

    const double acc_n_before = CalculateLongitudinalAcceleration(right_following.first, right_leading.first, right_following.second + ego_length + right_leading.second);
    const double acc_n_after = CalculateLongitudinalAcceleration(right_following.first, ego_agent, right_following.second);

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
        acc_threshold = benefit;
        behavior_pure_pursuit_.set_followed_line(right_corridor.first->get_center());
        is_changing_lane_ = true;
        target_corridor_ = right_corridor.first;
      }
    } else {
      std::cout << " To right would be unsafe: " << acc_n_after << " >= " << -safe_decel_ << std::endl;
    }
  }

  // Try change to left lane
  std::pair<world::map::DrivingCorridorPtr, bool> left_corridor = ego_agent->get_local_map()->get_left_adjacent_corridor(current_corridor_);
  if (left_corridor.second) {
    // std::cout << " Left: " << observed_world.get_map()->get_lane(left_corridor.first->get_lane_ids().front().second)->get_lane_position() << std::endl;

    std::pair<AgentPtr, double> left_following = observed_world.get_agent_behind(left_corridor.first);
    std::pair<AgentPtr, double> left_leading = observed_world.get_agent_in_front(left_corridor.first);

    if (left_following.first == nullptr) {
      std::cout << " Could go left, nobody there" << std::endl;
    }

    const double acc_c_after = CalculateLongitudinalAcceleration(ego_agent, left_leading.first, left_leading.second);

    if (asymmetric_passing_rules_ && left_leading.first != nullptr) {
      // Passing on the right is disallowed, therefore if the ego vehicle is faster than the leading vehicle on the left lane, its acceleration on the right
      // lane acc_c_before cannot be higher than its acceleration on the right lane acc_c_after

      float ego_velocity = ego_agent->get_current_state()(StateDefinition::VEL_POSITION);
      float left_leading_velocity = left_leading.first->get_current_state()(StateDefinition::VEL_POSITION);

      if (ego_velocity > left_leading_velocity && left_leading_velocity > critical_velocity_) {
        acc_c_before = std::min(acc_c_before, acc_c_after);
      }
    }

    const double acc_n_before = CalculateLongitudinalAcceleration(left_following.first, left_leading.first, left_following.second + ego_length + left_leading.second);
    const double acc_n_after = CalculateLongitudinalAcceleration(left_following.first, ego_agent, left_following.second);

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
        acc_threshold = benefit;
        behavior_pure_pursuit_.set_followed_line(left_corridor.first->get_center());
        is_changing_lane_ = true;
        target_corridor_ = left_corridor.first;
      }
    } else {
      std::cout << " To left would be unsafe: " << acc_n_after << std::endl;
    }
  }
}

void BehaviorMobil::ConcludeLaneChange(const world::ObservedWorld &observed_world) {
  using namespace geometry;
  using boost::geometry::get;
  using world::goal_definition::GoalDefinitionPolygon;

  Point2d ego_position = observed_world.current_ego_position();

  world::map::Frenet frenet_source_corridor = current_corridor_->FrenetFromCenterLine(ego_position);
  world::map::Frenet frenet_target_corridor = target_corridor_->FrenetFromCenterLine(ego_position);

  if (abs(frenet_target_corridor.lat) < abs(frenet_source_corridor.lat)) {
    is_changing_lane_ = false;
    current_corridor_ = target_corridor_;
    
    // Polygon goal_polygon = std::dynamic_pointer_cast<GoalDefinitionPolygon>(observed_world.get_ego_agent()->get_goal_definition())->get_shape();
    // Pose old_goal_pose = goal_polygon.center_;
    // Point2d new_goal_position = get_nearest_point(target_corridor_->get_center(), Point2d(old_goal_pose(0), old_goal_pose(1)));
    // Polygon new_goal_polygon = *std::dynamic_pointer_cast<Polygon>(goal_polygon.transform(
    //   Pose(get<0>(new_goal_position), get<1>(new_goal_polygon), old_goal_pose(2))));
    // world::goal_definition::GoalDefinitionPtr new_goal = std::make_shared<GoalDefinitionPolygon>(new_goal_polygon);
  }
}

double BehaviorMobil::CalculateLongitudinalAcceleration(const ConstAgentPtr &ego_agent, const ConstAgentPtr &leading_vehicle, const double distance) const {
  if (ego_agent == nullptr) {
    return 0.0;
  }

  // Obtain IDM parameters of the ego agent
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
    std::cout << "Using default" << std::endl;
    // default parameters
    desired_velocity = 15.0f;
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