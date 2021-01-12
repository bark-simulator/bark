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
#include "bark/world/tests/make_test_xodr_map.hpp"

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

    double vel_i = ego_agent->GetCurrentState()(StateDefinition::VEL_POSITION);

    double acc;
    if (leading_vehicle.first) {
      double net_distance =
          CalcNetDistance(observed_world, leading_vehicle.first, observed_world.GetLaneCorridor());
      State other_vehicle_state = leading_vehicle.first->GetCurrentState();
      double vel_other = other_vehicle_state(StateDefinition::VEL_POSITION);
      acc = CalcIDMAcc(net_distance, vel_i, vel_other);
    } else {
      acc = GetLonAccelerationMax() * CalcFreeRoadTerm(vel_i);
    }
    return acc;
  }

  virtual std::shared_ptr<BehaviorModel> Clone() const {
    std::shared_ptr<DummyBehaviorIDM> model_ptr =
        std::make_shared<DummyBehaviorIDM>(*this);
    return model_ptr;
  }
};

TEST(free_road_term, behavior_idm_classic) {
  auto params = std::make_shared<SetterParams>();
  DummyBehaviorIDM behavior(params);
  const double desired_velocity = behavior.GetDesiredVelocity();
  const double max_acceleration = behavior.GetLonAccelerationMax();
  const int exponent = behavior.GetExponent();

  // Create an observed world with specific goal definition and the
  // corresponding mcts state
  Polygon polygon = GenerateGoalRectangle(6, 3);
  std::shared_ptr<Polygon> goal_polygon(
      std::dynamic_pointer_cast<Polygon>(polygon.Translate(
          Point2d(50, -2))));  // < move the goal polygon into the driving
                               // corridor in front of the ego vehicle
  auto goal_definition_ptr =
      std::make_shared<GoalDefinitionPolygon>(*goal_polygon);

  double ego_velocity = desired_velocity, rel_distance = 7.0,
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
  const double max_acceleration = behavior.GetLonAccelerationMax();
  const double acc_lower_bound = behavior.GetAccelerationLimits().lon_acc_min;
  const double acc_upper_bound = behavior.GetAccelerationLimits().lon_acc_max;
  const double comfortable_braking_acceleration =
      behavior.GetComfortableBrakingAcceleration();

  // Create an observed world with specific goal definition and the
  // corresponding mcts state
  Polygon polygon = GenerateGoalRectangle(6, 3);
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
  const double desired_velocity = behavior.GetDesiredVelocity();

  // First case, we start with the desired velocity. After num steps, we should
  // advance
  double ego_velocity = desired_velocity, rel_distance = 7.0,
        velocity_difference = 0.0;
  double time_step = 0.2;
  int num_steps = 10;
  WorldPtr world =
      make_test_world(0, rel_distance, ego_velocity, velocity_difference);

  double x_start = world->GetAgents()
                      .begin()
                      ->second->GetCurrentState()[StateDefinition::X_POSITION];
  for (int i = 0; i < num_steps; ++i) {
    world->Step(time_step);
  }
  double x_end = world->GetAgents()
                    .begin()
                    ->second->GetCurrentState()[StateDefinition::X_POSITION];
  double x_diff_desired = x_start + ego_velocity * num_steps * time_step - x_end;
  EXPECT_NEAR(x_diff_desired, 0, 0.01);
}

TEST(drive_leading_vehicle, behavior_idm_classic) {
  auto params = std::make_shared<SetterParams>();
  // IDM Classic
  params->SetReal("BehaviorIDMClassic::MinimumSpacing",
                  1.0);  // Required for testing
  params->SetReal("BehaviorIDMClassic::DesiredTimeHeadway", 3.5);
  params->SetReal("BehaviorIDMClassic::MaxAcceleration",
                  1.0);  // Required for testing
  params->SetReal("BehaviorIDMClassic::AccelerationLowerBound", -1000.0);
  params->SetReal("BehaviorIDMClassic::AccelerationUpperBound", 1000.0);
  params->SetReal("BehaviorIDMClassic::DesiredVelocity", 8.0);
  params->SetReal("BehaviorIDMClassic::ComfortableBrakingAcceleration", 1.0);
  params->SetReal("BehaviorIDMClassic::MinVelocity", 0.0);
  params->SetReal("BehaviorIDMClassic::MaxVelocity", 50.0);
  BehaviorIDMClassic behavior(params);
  const double desired_velocity = behavior.GetDesiredVelocity();
  const double minimum_spacing = behavior.GetMinimumSpacing();
  const double desired_time_headway = behavior.GetDesiredTimeHeadway();

  // First case, we start with the desired velocity. After num steps, we should
  // advance
  double ego_velocity = desired_velocity, rel_distance = 5.0,
        velocity_difference = 4;
  double time_step = 0.02f;  // Very small time steps to verify differential
                            // integration character
  int num_steps = 1000;

  // Create an observed world with specific goal definition and the
  // corresponding mcts state
  Polygon polygon = GenerateGoalRectangle(6, 3);
  std::shared_ptr<Polygon> goal_polygon(
      std::dynamic_pointer_cast<Polygon>(polygon.Translate(
          Point2d(50, -2))));  // < move the goal polygon into the driving
                               // corridor in front of the ego vehicle
  auto goal_definition_ptr =
      std::make_shared<GoalDefinitionPolygon>(*goal_polygon);

  WorldPtr world = make_test_world(1, rel_distance, ego_velocity,
                                   velocity_difference, goal_definition_ptr);

  double v_start =
      world->GetAgents()
          .begin()
          ->second->GetCurrentState()[StateDefinition::VEL_POSITION];
  for (int i = 0; i < num_steps; ++i) {
    world->Step(time_step);
  }
  double v_end = world->GetAgents()
                    .begin()
                    ->second->GetCurrentState()[StateDefinition::VEL_POSITION];

  EXPECT_NEAR(v_end, ego_velocity - velocity_difference, 0.01);
}

TEST(coolness_factor_upper_eq_case, behavior_idm_classic) {
  auto params = std::make_shared<SetterParams>();
  // IDM Classic
  params->SetReal("BehaviorIDMClassic::MinimumSpacing",
                  1.0);  // Required for testing
  params->SetReal("BehaviorIDMClassic::DesiredTimeHeadway", 3.5);
  params->SetReal("BehaviorIDMClassic::MaxAcceleration",
                  1.0);  // Required for testing
  params->SetReal("BehaviorIDMClassic::AccelerationLowerBound", -1000.0);
  params->SetReal("BehaviorIDMClassic::AccelerationUpperBound", 1000.0);
  params->SetReal("BehaviorIDMClassic::DesiredVelocity", 8.0);
  params->SetReal("BehaviorIDMClassic::ComfortableBrakingAcceleration", 1.0);
  params->SetReal("BehaviorIDMClassic::MinVelocity", 0.0);
  params->SetReal("BehaviorIDMClassic::MaxVelocity", 50.0);
  params->SetBool("BehaviorIDMClassic::BrakeForLaneEnd", false);
  params->SetReal("BehaviorIDMClassic::BrakeForLaneEndEnabledDistance", 2.0);
  params->SetReal("BehaviorIDMClassic::BrakeForLaneEndDistanceOffset", 2.0);
  params->SetInt("BehaviorIDMClassic::NumTrajectoryTimePoints", 11);
  params->SetReal("BehaviorIDMClassic::CoolnessFactor", 0.89f);
  BehaviorIDMClassic behavior(params);
  const double desired_velocity = behavior.GetDesiredVelocity();
  const double minimum_spacing = behavior.GetMinimumSpacing();
  const double desired_time_headway = behavior.GetDesiredTimeHeadway();

  // First case, we start with the desired velocity. After num steps, we should
  // advance
  double ego_velocity = desired_velocity, rel_distance = 10.0,
        velocity_difference = -2, acc_ego = 2.0, acc_other = -5.0;
  double other_velocity = ego_velocity - velocity_difference;
  double time_step = 0.2;  // Very small time steps to verify differential
                           // integration character
  int num_steps = 1000;

  // Create an observed world with specific goal definition and the
  // corresponding mcts state
  Polygon polygon = GenerateGoalRectangle(6, 3);
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
  const double b = behavior.GetComfortableBrakingAcceleration();
  const double acc_cah_desired =
      ego_velocity * ego_velocity * acc_other /
      (other_velocity * other_velocity - 2 * rel_distance * acc_other);
  const double acc_idm_desired =
      behavior.CalcRawIDMAcc(rel_distance, ego_velocity, other_velocity);
  const double c = 0.89;
  const double acc_acc_desired =
      (1 - c) * acc_idm_desired +
      c * (acc_cah_desired + b * tanh((acc_idm_desired - acc_cah_desired) / b));
  EXPECT_NEAR(boost::get<Continuous1DAction>(action), acc_acc_desired, 0.001);
}

TEST(coolness_factor_lower_eq_case_vel_diff_neg, behavior_idm_classic) {
  auto params = std::make_shared<SetterParams>();
  // IDM Classic
  params->SetReal("BehaviorIDMClassic::MinimumSpacing",
                  1.0);  // Required for testing
  params->SetReal("BehaviorIDMClassic::DesiredTimeHeadway", 3.5);
  params->SetReal("BehaviorIDMClassic::MaxAcceleration",
                  1.0);  // Required for testing
  params->SetReal("BehaviorIDMClassic::AccelerationLowerBound", -1000.0);
  params->SetReal("BehaviorIDMClassic::AccelerationUpperBound", 1000.0);
  params->SetReal("BehaviorIDMClassic::DesiredVelocity", 8.0);
  params->SetReal("BehaviorIDMClassic::ComfortableBrakingAcceleration", 1.0);
  params->SetReal("BehaviorIDMClassic::MinVelocity", 0.0);
  params->SetReal("BehaviorIDMClassic::MaxVelocity", 50.0);
  params->SetBool("BehaviorIDMClassic::BrakeForLaneEnd", false);
  params->SetReal("BehaviorIDMClassic::BrakeForLaneEndEnabledDistance", 2.0);
  params->SetReal("BehaviorIDMClassic::BrakeForLaneEndDistanceOffset", 2.0);
  params->SetInt("BehaviorIDMClassic::NumTrajectoryTimePoints", 11);
  params->SetReal("BehaviorIDMClassic::CoolnessFactor", 0.6);
  BehaviorIDMClassic behavior(params);
  const double desired_velocity = behavior.GetDesiredVelocity();
  const double minimum_spacing = behavior.GetMinimumSpacing();
  const double desired_time_headway = behavior.GetDesiredTimeHeadway();

  // First case, we start with the desired velocity. After num steps, we should
  // advance
  double ego_velocity = desired_velocity, rel_distance = 20.0,
        velocity_difference = -1, acc_ego = 2.0, acc_other = 1.0;
  double other_velocity = ego_velocity - velocity_difference;
  double time_step = 0.2;  // Very small time steps to verify differential
                           // integration character

  Polygon polygon = GenerateGoalRectangle(6, 3);
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
  const double b = behavior.GetComfortableBrakingAcceleration();
  const double acc_cah_desired = acc_other;
  const double acc_idm_desired =
      behavior.CalcRawIDMAcc(rel_distance, ego_velocity, other_velocity);
  const double c = 0.6;
  const double acc_acc_desired =
      (1 - c) * acc_idm_desired +
      c * (acc_cah_desired + b * tanh((acc_idm_desired - acc_cah_desired) / b));
  EXPECT_NEAR(boost::get<Continuous1DAction>(action), acc_acc_desired, 0.001);
}

TEST(coolness_factor_lower_eq_case_vel_diff_pos, behavior_idm_classic) {
  auto params = std::make_shared<SetterParams>();
  // IDM Classic
  params->SetReal("BehaviorIDMClassic::MinimumSpacing",
                  1.0);  // Required for testing
  params->SetReal("BehaviorIDMClassic::DesiredTimeHeadway", 3.5);
  params->SetReal("BehaviorIDMClassic::MaxAcceleration",
                  1.0);  // Required for testing
  params->SetReal("BehaviorIDMClassic::AccelerationLowerBound", -1000.0);
  params->SetReal("BehaviorIDMClassic::AccelerationUpperBound", 1000.0);
  params->SetReal("BehaviorIDMClassic::DesiredVelocity", 8.0);
  params->SetReal("BehaviorIDMClassic::ComfortableBrakingAcceleration", 1.0);
  params->SetReal("BehaviorIDMClassic::MinVelocity", 0.0);
  params->SetReal("BehaviorIDMClassic::MaxVelocity", 50.0);
  params->SetReal("BehaviorIDMClassic::CoolnessFactor", 0.6);
  BehaviorIDMClassic behavior(params);
  const double desired_velocity = behavior.GetDesiredVelocity();
  const double minimum_spacing = behavior.GetMinimumSpacing();
  const double desired_time_headway = behavior.GetDesiredTimeHeadway();

  // First case, we start with the desired velocity. After num steps, we should
  // advance
  double ego_velocity = desired_velocity, rel_distance = 5.0,
        velocity_difference = 5, acc_ego = 2.0, acc_other = 1.0;
  double other_velocity = ego_velocity - velocity_difference;
  double time_step = 0.2;  // Very small time steps to verify differential
                           // integration character
  int num_steps = 1000;

  // Create an observed world with specific goal definition and the
  // corresponding mcts state
  Polygon polygon = GenerateGoalRectangle(6, 3);
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
  const double b = behavior.GetComfortableBrakingAcceleration();
  const double acc_cah_desired = acc_other - velocity_difference *
                                                velocity_difference * 1.0 /
                                                (2.0 * rel_distance);
  const double acc_idm_desired =
      behavior.CalcRawIDMAcc(rel_distance, ego_velocity, other_velocity);
  const double c = 0.6;
  const double acc_acc_desired =
      (1 - c) * acc_idm_desired +
      c * (acc_cah_desired + b * tanh((acc_idm_desired - acc_cah_desired) / b));
  EXPECT_NEAR(boost::get<Continuous1DAction>(action), acc_acc_desired, 0.001);
}

TEST(CalcNetDistance, behavior_idm_classic) {
  auto params = std::make_shared<SetterParams>();

  OpenDriveMapPtr open_drive_map =
      bark::world::tests::MakeXodrMapTwoRoadsOneLane();

  MapInterfacePtr map_interface = std::make_shared<MapInterface>();
  map_interface->interface_from_opendrive(open_drive_map);

  Polygon shape = bark::geometry::standard_shapes::CarRectangle();

  Polygon polygon = GenerateGoalRectangle(6, 3);
  std::shared_ptr<Polygon> goal_polygon(
      std::dynamic_pointer_cast<Polygon>(polygon.Translate(Point2d(90, -2))));

  auto goal_definition_ptr =
      std::make_shared<GoalDefinitionPolygon>(*goal_polygon);

  // agent1 is placed on road 1
  State init_state1(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state1 << 0.0, 20, -1.75, 0.0, 10.0;
  auto b1 = std::make_shared<BehaviorIDMClassic>(params);
  AgentPtr agent1(new Agent(init_state1, b1, nullptr, nullptr, shape,
                            params, goal_definition_ptr, map_interface,
                            bark::geometry::Model3D()));

  // agent2 is placed on road 2
  State init_state2(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state2 << 0.0, 70, -1.75, 0.0, 10.0;
  auto b2 = std::make_shared<BehaviorIDMClassic>(params);
  AgentPtr agent2(new Agent(init_state2, b2, nullptr, nullptr, shape,
                            params, goal_definition_ptr, map_interface,
                            bark::geometry::Model3D()));

  WorldPtr world(new World(params));
  world->AddAgent(agent1);
  world->AddAgent(agent2);
  world->UpdateAgentRTree();

  world->SetMap(map_interface);

  auto current_world_state = WorldPtr(world->Clone());
  ObservedWorld observed_world(
      current_world_state,
      current_world_state->GetAgents().begin()->second->GetAgentId());

  BehaviorIDMClassic behavior(params);
  double distance = behavior.CalcNetDistance(observed_world, agent2, observed_world.GetLaneCorridor());
  double x_diff = init_state2(StateDefinition::X_POSITION) -
                 init_state1(StateDefinition::X_POSITION);
  double distance_expected =
      x_diff - agent1->GetShape().front_dist_ - agent2->GetShape().rear_dist_;

  EXPECT_EQ(distance, distance_expected);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}