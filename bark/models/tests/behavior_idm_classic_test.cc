// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <Eigen/Core>
#include "gtest/gtest.h"

#include "bark/commons/params/setter_params.hpp"
#include "bark/geometry/commons.hpp"
#include "bark/geometry/line.hpp"
#include "bark/geometry/polygon.hpp"
#include "bark/models/behavior/constant_acceleration/constant_acceleration.hpp"
#include "bark/models/behavior/idm/idm_classic.hpp"
#include "bark/models/execution/interpolation/interpolate.hpp"
#include "bark/world/observed_world.hpp"
#include "bark/world/tests/make_test_world.hpp"

using namespace bark::models::dynamic;
using namespace bark::models::execution;
using namespace bark::commons;
using namespace bark::models::behavior;
using namespace bark::world::map;
using namespace bark::models::dynamic;
using namespace bark::world;
using namespace bark::geometry;
using namespace bark::world::tests;

using bark::geometry::standard_shapes::GenerateGoalRectangle;

class DummyBehaviorIDM : public BehaviorIDMClassic {
 public:
  DummyBehaviorIDM(const ParamsPtr& params)
      : BehaviorIDMClassic(params), BehaviorModel(params) {}
  virtual double CalculateLongitudinalAcceleration(
      const ObservedWorld& observed_world) {
    std::pair<AgentPtr, FrenetPosition> leading_vehicle =
        observed_world.GetAgentInFront();
    std::shared_ptr<const Agent> ego_agent = observed_world.GetEgoAgent();

    float vel_i = ego_agent->GetCurrentState()(StateDefinition::VEL_POSITION);

    double acc;
    if (leading_vehicle.first) {
      double net_distance = CalcNetDistance(ego_agent, leading_vehicle.first);
      State other_vehicle_state = leading_vehicle.first->GetCurrentState();
      double vel_other = other_vehicle_state(StateDefinition::VEL_POSITION);
      acc = CalcIDMAcc(net_distance, vel_i, vel_other);
    } else {
      acc = GetMaxAcceleration() * CalcFreeRoadTerm(vel_i);
    }
    return acc;
  }

  virtual std::shared_ptr<BehaviorModel> Clone() const {
    std::shared_ptr<DummyBehaviorIDM> model_ptr =
        std::make_shared<DummyBehaviorIDM>(*this);
    return std::dynamic_pointer_cast<BehaviorModel>(model_ptr);
  }
};

TEST(free_road_term, behavior_idm_classic) {
  auto params = std::make_shared<SetterParams>();
  DummyBehaviorIDM behavior(params);
  const float desired_velocity = behavior.GetDesiredVelocity();
  const float max_acceleration = behavior.GetMaxAcceleration();
  const int exponent = behavior.GetExponent();

  // Create an observed world with specific goal definition and the
  // corresponding mcts state
  Polygon polygon = GenerateGoalRectangle(6,3);
  std::shared_ptr<Polygon> goal_polygon(
      std::dynamic_pointer_cast<Polygon>(polygon.Translate(
          Point2d(50, -2))));  // < move the goal polygon into the driving
                               // corridor in front of the ego vehicle
  auto goal_definition_ptr =
      std::make_shared<GoalDefinitionPolygon>(*goal_polygon);

  float ego_velocity = desired_velocity, rel_distance = 7.0,
        velocity_difference = 0.0;
  auto observed_world = make_test_observed_world(
      0, rel_distance, ego_velocity, velocity_difference, goal_definition_ptr);
  double idm_acceleration =
      behavior.CalculateLongitudinalAcceleration(observed_world);

  // no vehicle is in front only free road term should give acceleration
  double desired_acceleration =
      max_acceleration * (1 - pow(ego_velocity / desired_velocity, exponent));
  EXPECT_NEAR(idm_acceleration, desired_acceleration, 0.0001);

  ego_velocity = desired_velocity + 10;
  observed_world = make_test_observed_world(0, rel_distance, ego_velocity,
                                            velocity_difference);
  idm_acceleration = behavior.CalculateLongitudinalAcceleration(observed_world);

  // no vehicle is in front only free road term should give acceleration
  desired_acceleration =
      max_acceleration * (1 - pow(ego_velocity / desired_velocity, exponent));
  EXPECT_NEAR(idm_acceleration, desired_acceleration, 0.0001);

  ego_velocity = desired_velocity - 12;
  observed_world = make_test_observed_world(0, rel_distance, ego_velocity,
                                            velocity_difference);
  idm_acceleration = behavior.CalculateLongitudinalAcceleration(observed_world);

  // no vehicle is in front only free road term should give acceleration
  desired_acceleration =
      max_acceleration * (1 - pow(ego_velocity / desired_velocity, exponent));
  EXPECT_NEAR(idm_acceleration, desired_acceleration, 0.0001);
}

TEST(interaction_term, behavior_idm_classic) {
  auto params = std::make_shared<SetterParams>();
  DummyBehaviorIDM behavior(params);
  const double desired_velocity = behavior.GetDesiredVelocity();
  const double minimum_spacing = behavior.GetMinimumSpacing();
  const double desired_time_headway = behavior.GetDesiredTimeHeadway();
  const double max_acceleration = behavior.GetMaxAcceleration();
  const double acc_lower_bound = behavior.GetAccelerationLowerBound();
  const double acc_upper_bound = behavior.GetAccelerationUpperBound();
  const double comfortable_braking_acceleration =
      behavior.GetComfortableBrakingAcceleration();

  // Create an observed world with specific goal definition and the
  // corresponding mcts state
  Polygon polygon = GenerateGoalRectangle(6,3);
  std::shared_ptr<Polygon> goal_polygon(
      std::dynamic_pointer_cast<Polygon>(polygon.Translate(
          Point2d(50, -2))));  // < move the goal polygon into the driving
                               // corridor in front of the ego vehicle
  auto goal_definition_ptr =
      std::make_shared<GoalDefinitionPolygon>(*goal_polygon);

  // vehicle is in front, zero velocity equal desired velocity thus only
  // interaction term
  double ego_velocity = desired_velocity, rel_distance = 7.0,
         velocity_difference = 0.0;
  auto observed_world = make_test_observed_world(
      1, rel_distance, ego_velocity, velocity_difference, goal_definition_ptr);
  double idm_acceleration =
      behavior.CalculateLongitudinalAcceleration(observed_world);
  double helper_state =
      minimum_spacing + ego_velocity * desired_time_headway +
      ego_velocity * velocity_difference /
          (2 * sqrt(max_acceleration * comfortable_braking_acceleration));
  double desired_acceleration = std::max(
      acc_lower_bound, -max_acceleration * pow(helper_state / rel_distance, 2));
  EXPECT_NEAR(idm_acceleration, desired_acceleration, 0.001);

  // velocity difference to other vehicle
  ego_velocity = desired_velocity, rel_distance = 7.0,
  velocity_difference = 5.0;
  observed_world = make_test_observed_world(
      1, rel_distance, ego_velocity, velocity_difference, goal_definition_ptr);
  idm_acceleration = behavior.CalculateLongitudinalAcceleration(observed_world);
  helper_state =
      minimum_spacing + ego_velocity * desired_time_headway +
      ego_velocity * velocity_difference /
          (2 * sqrt(max_acceleration * comfortable_braking_acceleration));
  desired_acceleration = std::max(
      acc_lower_bound, -max_acceleration * pow(helper_state / rel_distance, 2));
  EXPECT_NEAR(idm_acceleration, desired_acceleration, 0.001);
}

TEST(drive_free, behavior_idm_classic) {
  auto params = std::make_shared<SetterParams>();
  DummyBehaviorIDM behavior(params);
  const float desired_velocity = behavior.GetDesiredVelocity();

  // First case, we start with the desired velocity. After num steps, we should
  // advance
  float ego_velocity = desired_velocity, rel_distance = 7.0,
        velocity_difference = 0.0;
  float time_step = 0.2f;
  int num_steps = 10;
  WorldPtr world =
      make_test_world(0, rel_distance, ego_velocity, velocity_difference);

  float x_start = world->GetAgents()
                      .begin()
                      ->second->GetCurrentState()[StateDefinition::X_POSITION];
  for (int i = 0; i < num_steps; ++i) {
    world->Step(time_step);
  }
  float x_end = world->GetAgents()
                    .begin()
                    ->second->GetCurrentState()[StateDefinition::X_POSITION];
  float x_diff_desired = x_start + ego_velocity * num_steps * time_step - x_end;
  EXPECT_NEAR(x_diff_desired, 0, 0.01);
}

TEST(drive_leading_vehicle, behavior_idm_classic) {
  auto params = std::make_shared<SetterParams>();
  // IDM Classic
  params->SetReal("BehaviorIDMClassic::MinimumSpacing",
                  1.0f);  // Required for testing
  params->SetReal("BehaviorIDMClassic::DesiredTimeHeadway", 3.5);
  params->SetReal("BehaviorIDMClassic::MaxAcceleration",
                  1.0f);  // Required for testing
  params->SetReal("BehaviorIDMClassic::AccelerationLowerBound", -1000.0);
  params->SetReal("BehaviorIDMClassic::AccelerationUpperBound", 1000.0);
  params->SetReal("BehaviorIDMClassic::DesiredVelocity", 8.0f);
  params->SetReal("BehaviorIDMClassic::ComfortableBrakingAcceleration", 1.0f);
  params->SetReal("BehaviorIDMClassic::MinVelocity", 0.0f);
  params->SetReal("BehaviorIDMClassic::MaxVelocity", 50.0f);
  BehaviorIDMClassic behavior(params);
  const float desired_velocity = behavior.GetDesiredVelocity();
  const float minimum_spacing = behavior.GetMinimumSpacing();
  const float desired_time_headway = behavior.GetDesiredTimeHeadway();

  // First case, we start with the desired velocity. After num steps, we should
  // advance
  float ego_velocity = desired_velocity, rel_distance = 5.0,
        velocity_difference = 4;
  float time_step = 0.02f;  // Very small time steps to verify differential
                            // integration character
  int num_steps = 1000;

  // Create an observed world with specific goal definition and the
  // corresponding mcts state
  Polygon polygon = GenerateGoalRectangle(6,3);
  std::shared_ptr<Polygon> goal_polygon(
      std::dynamic_pointer_cast<Polygon>(polygon.Translate(
          Point2d(50, -2))));  // < move the goal polygon into the driving
                               // corridor in front of the ego vehicle
  auto goal_definition_ptr =
      std::make_shared<GoalDefinitionPolygon>(*goal_polygon);

  WorldPtr world = make_test_world(1, rel_distance, ego_velocity,
                                   velocity_difference, goal_definition_ptr);

  float v_start =
      world->GetAgents()
          .begin()
          ->second->GetCurrentState()[StateDefinition::VEL_POSITION];
  for (int i = 0; i < num_steps; ++i) {
    world->Step(time_step);
  }
  float v_end = world->GetAgents()
                    .begin()
                    ->second->GetCurrentState()[StateDefinition::VEL_POSITION];

  EXPECT_NEAR(v_end, ego_velocity - velocity_difference, 0.01);
}

TEST(coolness_factor_upper_eq_case, behavior_idm_classic) {
  auto params = std::make_shared<SetterParams>();
  // IDM Classic
  params->SetReal("BehaviorIDMClassic::MinimumSpacing",
                  1.0f);  // Required for testing
  params->SetReal("BehaviorIDMClassic::DesiredTimeHeadway", 3.5);
  params->SetReal("BehaviorIDMClassic::MaxAcceleration",
                  1.0f);  // Required for testing
  params->SetReal("BehaviorIDMClassic::AccelerationLowerBound", -1000.0);
  params->SetReal("BehaviorIDMClassic::AccelerationUpperBound", 1000.0);
  params->SetReal("BehaviorIDMClassic::DesiredVelocity", 8.0f);
  params->SetReal("BehaviorIDMClassic::ComfortableBrakingAcceleration", 1.0f);
  params->SetReal("BehaviorIDMClassic::MinVelocity", 0.0f);
  params->SetReal("BehaviorIDMClassic::MaxVelocity", 50.0f);
  params->SetBool("BehaviorIDMClassic::BrakeForLaneEnd", false);
  params->SetReal("BehaviorIDMClassic::BrakeForLaneEndEnabledDistance", 2.0f);
  params->SetReal("BehaviorIDMClassic::BrakeForLaneEndDistanceOffset", 2.0f);
  params->SetInt("BehaviorIDMClassic::NumTrajectoryTimePoints", 11);
  params->SetReal("BehaviorIDMClassic::CoolnessFactor", 0.89f);
  BehaviorIDMClassic behavior(params);
  const float desired_velocity = behavior.GetDesiredVelocity();
  const float minimum_spacing = behavior.GetMinimumSpacing();
  const float desired_time_headway = behavior.GetDesiredTimeHeadway();

  // First case, we start with the desired velocity. After num steps, we should
  // advance
  float ego_velocity = desired_velocity, rel_distance = 10.0,
        velocity_difference = -2, acc_ego = 2.0f, acc_other = -5.0f;
  float other_velocity = ego_velocity - velocity_difference;
  float time_step = 0.2f;  // Very small time steps to verify differential
                           // integration character
  int num_steps = 1000;

  // Create an observed world with specific goal definition and the
  // corresponding mcts state
  Polygon polygon = GenerateGoalRectangle(6,3);
  std::shared_ptr<Polygon> goal_polygon(
      std::dynamic_pointer_cast<Polygon>(polygon.Translate(
          Point2d(50, -2))));  // < move the goal polygon into the driving
                               // corridor in front of the ego vehicle
  auto goal_definition_ptr =
      std::make_shared<GoalDefinitionPolygon>(*goal_polygon);

  auto observed_world = make_test_observed_world(
      1, rel_distance, ego_velocity, velocity_difference, goal_definition_ptr,
      acc_ego, acc_other);

  auto traj = behavior.Plan(time_step, observed_world);
  auto action = behavior.GetLastAction();

  // upper case of equation 11.25
  const float b = behavior.GetComfortableBrakingAcceleration();
  const float acc_cah_desired =
      ego_velocity * ego_velocity * acc_other /
      (other_velocity * other_velocity - 2 * rel_distance * acc_other);
  const float acc_idm_desired =
      behavior.CalcRawIDMAcc(rel_distance, ego_velocity, other_velocity);
  const float c = 0.89;
  const float acc_acc_desired =
      (1 - c) * acc_idm_desired +
      c * (acc_cah_desired + b * tanh((acc_idm_desired - acc_cah_desired) / b));
  EXPECT_NEAR(boost::get<Continuous1DAction>(action), acc_acc_desired, 0.001);
}

TEST(coolness_factor_lower_eq_case_vel_diff_neg, behavior_idm_classic) {
  auto params = std::make_shared<SetterParams>();
  // IDM Classic
  params->SetReal("BehaviorIDMClassic::MinimumSpacing",
                  1.0f);  // Required for testing
  params->SetReal("BehaviorIDMClassic::DesiredTimeHeadway", 3.5);
  params->SetReal("BehaviorIDMClassic::MaxAcceleration",
                  1.0f);  // Required for testing
  params->SetReal("BehaviorIDMClassic::AccelerationLowerBound", -1000.0);
  params->SetReal("BehaviorIDMClassic::AccelerationUpperBound", 1000.0);
  params->SetReal("BehaviorIDMClassic::DesiredVelocity", 8.0f);
  params->SetReal("BehaviorIDMClassic::ComfortableBrakingAcceleration", 1.0f);
  params->SetReal("BehaviorIDMClassic::MinVelocity", 0.0f);
  params->SetReal("BehaviorIDMClassic::MaxVelocity", 50.0f);
  params->SetBool("BehaviorIDMClassic::BrakeForLaneEnd", false);
  params->SetReal("BehaviorIDMClassic::BrakeForLaneEndEnabledDistance", 2.0f);
  params->SetReal("BehaviorIDMClassic::BrakeForLaneEndDistanceOffset", 2.0f);
  params->SetInt("BehaviorIDMClassic::NumTrajectoryTimePoints", 11);
  params->SetReal("BehaviorIDMClassic::CoolnessFactor", 0.6f);
  BehaviorIDMClassic behavior(params);
  const float desired_velocity = behavior.GetDesiredVelocity();
  const float minimum_spacing = behavior.GetMinimumSpacing();
  const float desired_time_headway = behavior.GetDesiredTimeHeadway();

  // First case, we start with the desired velocity. After num steps, we should
  // advance
  float ego_velocity = desired_velocity, rel_distance = 20.0,
        velocity_difference = -1, acc_ego = 2.0f, acc_other = 1.0f;
  float other_velocity = ego_velocity - velocity_difference;
  float time_step = 0.2f;  // Very small time steps to verify differential
                           // integration character

  Polygon polygon = GenerateGoalRectangle(6,3);
  std::shared_ptr<Polygon> goal_polygon(
      std::dynamic_pointer_cast<Polygon>(polygon.Translate(
          Point2d(50, -2))));  // < move the goal polygon into the driving
                               // corridor in front of the ego vehicle
  auto goal_definition_ptr =
      std::make_shared<GoalDefinitionPolygon>(*goal_polygon);

  auto observed_world = make_test_observed_world(
      1, rel_distance, ego_velocity, velocity_difference, goal_definition_ptr,
      acc_ego, acc_other);

  auto traj = behavior.Plan(time_step, observed_world);
  auto action = behavior.GetLastAction();

  // upper case of equation 11.25
  const float b = behavior.GetComfortableBrakingAcceleration();
  const float acc_cah_desired = acc_other;
  const float acc_idm_desired =
      behavior.CalcRawIDMAcc(rel_distance, ego_velocity, other_velocity);
  const float c = 0.6;
  const float acc_acc_desired =
      (1 - c) * acc_idm_desired +
      c * (acc_cah_desired + b * tanh((acc_idm_desired - acc_cah_desired) / b));
  EXPECT_NEAR(boost::get<Continuous1DAction>(action), acc_acc_desired, 0.001);
}

TEST(coolness_factor_lower_eq_case_vel_diff_pos, behavior_idm_classic) {
  auto params = std::make_shared<SetterParams>();
  // IDM Classic
  params->SetReal("BehaviorIDMClassic::MinimumSpacing",
                  1.0f);  // Required for testing
  params->SetReal("BehaviorIDMClassic::DesiredTimeHeadway", 3.5);
  params->SetReal("BehaviorIDMClassic::MaxAcceleration",
                  1.0f);  // Required for testing
  params->SetReal("BehaviorIDMClassic::AccelerationLowerBound", -1000.0);
  params->SetReal("BehaviorIDMClassic::AccelerationUpperBound", 1000.0);
  params->SetReal("BehaviorIDMClassic::DesiredVelocity", 8.0f);
  params->SetReal("BehaviorIDMClassic::ComfortableBrakingAcceleration", 1.0f);
  params->SetReal("BehaviorIDMClassic::MinVelocity", 0.0f);
  params->SetReal("BehaviorIDMClassic::MaxVelocity", 50.0f);
  params->SetReal("BehaviorIDMClassic::CoolnessFactor", 0.6f);
  BehaviorIDMClassic behavior(params);
  const float desired_velocity = behavior.GetDesiredVelocity();
  const float minimum_spacing = behavior.GetMinimumSpacing();
  const float desired_time_headway = behavior.GetDesiredTimeHeadway();

  // First case, we start with the desired velocity. After num steps, we should
  // advance
  float ego_velocity = desired_velocity, rel_distance = 5.0,
        velocity_difference = 5, acc_ego = 2.0f, acc_other = 1.0f;
  float other_velocity = ego_velocity - velocity_difference;
  float time_step = 0.2f;  // Very small time steps to verify differential
                           // integration character
  int num_steps = 1000;

  // Create an observed world with specific goal definition and the
  // corresponding mcts state
  Polygon polygon = GenerateGoalRectangle(6,3);
  std::shared_ptr<Polygon> goal_polygon(
      std::dynamic_pointer_cast<Polygon>(polygon.Translate(
          Point2d(50, -2))));  // < move the goal polygon into the driving
                               // corridor in front of the ego vehicle
  auto goal_definition_ptr =
      std::make_shared<GoalDefinitionPolygon>(*goal_polygon);

  auto observed_world = make_test_observed_world(
      1, rel_distance, ego_velocity, velocity_difference, goal_definition_ptr,
      acc_ego, acc_other);

  auto traj = behavior.Plan(time_step, observed_world);
  auto action = behavior.GetLastAction();

  // upper case of equation 11.25
  const float b = behavior.GetComfortableBrakingAcceleration();
  const float acc_cah_desired = acc_other - velocity_difference *
                                                velocity_difference * 1.0 /
                                                (2.0 * rel_distance);
  const float acc_idm_desired =
      behavior.CalcRawIDMAcc(rel_distance, ego_velocity, other_velocity);
  const float c = 0.6;
  const float acc_acc_desired =
      (1 - c) * acc_idm_desired +
      c * (acc_cah_desired + b * tanh((acc_idm_desired - acc_cah_desired) / b));
  EXPECT_NEAR(boost::get<Continuous1DAction>(action), acc_acc_desired, 0.001);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}