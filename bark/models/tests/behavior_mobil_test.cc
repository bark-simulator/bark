// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <Eigen/Core>
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "gtest/gtest.h"

#include "bark/commons/params/setter_params.hpp"
#include "bark/geometry/polygon.hpp"
#include "bark/geometry/standard_shapes.hpp"
#include "bark/models/behavior/constant_acceleration/constant_acceleration.hpp"
#include "bark/models/behavior/rule_based/mobil.hpp"
#include "bark/models/behavior/rule_based/mobil_behavior.hpp"
#include "bark/models/dynamic/single_track.hpp"
#include "bark/models/execution/interpolation/interpolate.hpp"
#include "bark/world/goal_definition/goal_definition_polygon.hpp"
#include "bark/world/objects/agent.hpp"
#include "bark/world/observed_world.hpp"
#include "bark/world/tests/make_test_xodr_map.hpp"

using namespace bark::models::dynamic;
using namespace bark::models::execution;
using namespace bark::commons;
using namespace bark::models::behavior;
using namespace bark::world::map;
using namespace bark::models::dynamic;

using bark::geometry::Model3D;
using bark::geometry::Point2d;
using bark::geometry::Polygon;
using bark::geometry::Pose;
using bark::geometry::standard_shapes::CarRectangle;
using bark::geometry::standard_shapes::GenerateGoalRectangle;
using bark::world::ObservedWorld;
using bark::world::World;
using bark::world::WorldPtr;
using bark::world::goal_definition::GoalDefinitionPolygon;
using bark::world::objects::Agent;
using bark::world::objects::AgentPtr;
using bark::world::tests::MakeXodrMapEndingLaneInParallel;
using bark::world::tests::MakeXodrMapOneRoadTwoLanes;

ObservedWorld make_observed_world_mobil(double vel, ParamsPtr params) {
  // Setting Up Map
  auto open_drive_map = MakeXodrMapOneRoadTwoLanes();
  auto map_interface = std::make_shared<MapInterface>();
  map_interface->interface_from_opendrive(open_drive_map);

  Polygon car_polygon = CarRectangle();

  Polygon polygon = GenerateGoalRectangle(6, 3);
  std::shared_ptr<Polygon> goal_polygon(
      std::dynamic_pointer_cast<Polygon>(polygon.Translate(
          Point2d(100, -2))));  // < move the goal polygon into the driving
                                // corridor in front of the ego vehicle
  auto goal_definition_ptr =
      std::make_shared<GoalDefinitionPolygon>(*goal_polygon);

  ExecutionModelPtr exec_model(new ExecutionModelInterpolate(params));
  DynamicModelPtr dyn_model(new SingleTrackModel(params));
  BehaviorModelPtr beh_model(new BehaviorMobilRuleBased(params));

  State init_state1(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state1 << 0.0, 53.0, -1.75, 0.0, vel;
  AgentPtr agent1(new Agent(init_state1, beh_model, dyn_model, exec_model,
                            car_polygon, params, goal_definition_ptr,
                            map_interface, bark::geometry::Model3D()));

  // Preceding Agent
  ExecutionModelPtr exec_model2(new ExecutionModelInterpolate(params));
  DynamicModelPtr dyn_model2(new SingleTrackModel(params));
  BehaviorModelPtr beh_model2(new BehaviorConstantAcceleration(params));

  State init_state2(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state2 << 0.0, 80.0, -1.75, 0.0, vel;
  AgentPtr agent2(new Agent(init_state2, beh_model2, dyn_model2, exec_model2,
                            car_polygon, params, goal_definition_ptr,
                            map_interface, bark::geometry::Model3D()));

  // Agent coming from behind on the right
  ExecutionModelPtr exec_model3(new ExecutionModelInterpolate(params));
  DynamicModelPtr dyn_model3(new SingleTrackModel(params));
  BehaviorModelPtr beh_model3(new BehaviorConstantAcceleration(params));

  State init_state3(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state3 << 0.0, 43.0, -1.75 - 3.5, 0.0, vel;
  AgentPtr agent3(new Agent(init_state3, beh_model3, dyn_model3, exec_model3,
                            car_polygon, params, goal_definition_ptr,
                            map_interface, bark::geometry::Model3D()));

  // Following Agent
  ExecutionModelPtr exec_model4(new ExecutionModelInterpolate(params));
  DynamicModelPtr dyn_model4(new SingleTrackModel(params));
  BehaviorModelPtr beh_model4(new BehaviorConstantAcceleration(params));

  State init_state4(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state4 << 0.0, 20.0, -1.75, 0.0, vel;
  AgentPtr agent4(new Agent(init_state4, beh_model4, dyn_model4, exec_model4,
                            car_polygon, params, goal_definition_ptr,
                            map_interface, bark::geometry::Model3D()));

  // Agent on the right in front
  ExecutionModelPtr exec_model5(new ExecutionModelInterpolate(params));
  DynamicModelPtr dyn_model5(new SingleTrackModel(params));
  BehaviorModelPtr beh_model5(new BehaviorConstantAcceleration(params));

  State init_state5(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state5 << 0.0, 100.0, -1.75 - 3.5, 0.0, vel;
  AgentPtr agent5(new Agent(init_state5, beh_model5, dyn_model5, exec_model5,
                            car_polygon, params, goal_definition_ptr,
                            map_interface, bark::geometry::Model3D()));

  // Construct World
  WorldPtr world(new World(params));
  world->AddAgent(agent1);
  world->AddAgent(agent2);
  world->AddAgent(agent3);
  world->AddAgent(agent4);
  world->AddAgent(agent5);
  world->UpdateAgentRTree();

  WorldPtr current_world_state(world->Clone());
  ObservedWorld observed_world(current_world_state, agent1->GetAgentId());

  return observed_world;
}

TEST(safety_not_met, behavior_mobil) {
  double vel = 5.0;
  auto params = std::make_shared<SetterParams>();
  params->SetReal("BehaviorIDMClassic::DesiredVelocity", vel);
  params->SetReal("BehaviorMobilRuleBased::AThr", 0.2);
  params->SetReal("BehaviorMobilRuleBased::Politeness", 0.0);
  params->SetReal("BehaviorMobilRuleBased::BSafe", 1.0);

  ObservedWorld observed_world = make_observed_world_mobil(vel, params);

  const BehaviorModelPtr behavior_model =
      observed_world.GetEgoAgent()->GetBehaviorModel();
  auto behavior_mobil =
      std::dynamic_pointer_cast<BehaviorMobilRuleBased>(behavior_model);
  behavior_mobil->SetLaneCorridor(observed_world.GetLaneCorridor());

  LaneChangeDecision decision;
  LaneCorridorPtr lane_corr;
  std::tie(decision, lane_corr) =
      behavior_mobil->CheckIfLaneChangeBeneficial(observed_world);
  EXPECT_EQ(decision, LaneChangeDecision::KeepLane);
  BARK_EXPECT_TRUE(lane_corr != nullptr);
}

TEST(impolite_incentive_met_safety_met, behavior_mobil) {
  double vel_ego = 5.0;
  auto params = std::make_shared<SetterParams>();
  params->SetReal("BehaviorIDMClassic::DesiredVelocity", vel_ego);
  params->SetReal("BehaviorMobilRuleBased::AThr", 0.1);
  params->SetReal("BehaviorMobilRuleBased::Politeness", 0.0);
  params->SetReal("BehaviorMobilRuleBased::BSafe", 4.0);

  ObservedWorld observed_world = make_observed_world_mobil(vel_ego, params);

  const BehaviorModelPtr behavior_model =
      observed_world.GetEgoAgent()->GetBehaviorModel();
  auto behavior_mobil =
      std::dynamic_pointer_cast<BehaviorMobilRuleBased>(behavior_model);
  behavior_mobil->SetLaneCorridor(observed_world.GetLaneCorridor());

  LaneChangeDecision decision;
  LaneCorridorPtr lane_corr;
  std::tie(decision, lane_corr) =
      behavior_mobil->CheckIfLaneChangeBeneficial(observed_world);
  EXPECT_EQ(decision, LaneChangeDecision::ChangeLane);
  BARK_EXPECT_TRUE(lane_corr != nullptr);
}

TEST(polite_incentive_not_met_safety_met, behavior_mobil) {
  double vel_ego = 5.0;
  auto params = std::make_shared<SetterParams>();
  params->SetReal("BehaviorIDMClassic::DesiredVelocity", vel_ego);
  params->SetReal("BehaviorMobilRuleBased::AThr", 0.2);
  params->SetReal("BehaviorMobilRuleBased::Politeness", 1.0);
  params->SetReal("BehaviorMobilRuleBased::BSafe", 4.0);

  ObservedWorld observed_world = make_observed_world_mobil(vel_ego, params);

  const BehaviorModelPtr behavior_model =
      observed_world.GetEgoAgent()->GetBehaviorModel();
  auto behavior_mobil =
      std::dynamic_pointer_cast<BehaviorMobilRuleBased>(behavior_model);
  behavior_mobil->SetLaneCorridor(observed_world.GetLaneCorridor());

  LaneChangeDecision decision;
  LaneCorridorPtr lane_corr;
  std::tie(decision, lane_corr) =
      behavior_mobil->CheckIfLaneChangeBeneficial(observed_world);
  EXPECT_EQ(decision, LaneChangeDecision::KeepLane);
  BARK_EXPECT_TRUE(lane_corr != nullptr);
}

TEST(polite_incentive_met_safety_met, behavior_mobil) {
  double vel_ego = 5.0;
  auto params = std::make_shared<SetterParams>();
  params->SetReal("BehaviorIDMClassic::DesiredVelocity", vel_ego);
  params->SetReal("BehaviorMobilRuleBased::AThr", -5.0);  // HACK
  params->SetReal("BehaviorMobilRuleBased::Politeness", 1.0);
  params->SetReal("BehaviorMobilRuleBased::BSafe", 4.0);

  ObservedWorld observed_world = make_observed_world_mobil(vel_ego, params);

  const BehaviorModelPtr behavior_model =
      observed_world.GetEgoAgent()->GetBehaviorModel();
  auto behavior_mobil =
      std::dynamic_pointer_cast<BehaviorMobilRuleBased>(behavior_model);
  behavior_mobil->SetLaneCorridor(observed_world.GetLaneCorridor());

  LaneChangeDecision decision;
  LaneCorridorPtr lane_corr;
  std::tie(decision, lane_corr) =
      behavior_mobil->CheckIfLaneChangeBeneficial(observed_world);
  EXPECT_EQ(decision, LaneChangeDecision::ChangeLane);
  BARK_EXPECT_TRUE(lane_corr != nullptr);
}

TEST(change_lane_due_to_lane_ending, behavior_mobil) {
  double vel_ego = 5.0;
  auto params = std::make_shared<SetterParams>();
  params->SetReal("BehaviorIDMClassic::DesiredVelocity", vel_ego);
  params->SetBool("BehaviorIDMClassic::BrakeForLaneEnd", true);
  params->SetReal("BehaviorIDMClassic::BrakeForLaneEndEnabledDistance", 60);
  params->SetReal("BehaviorIDMClassic::MaxAcceleration", 1.9);
  params->SetReal("BehaviorMobilRuleBased::AThr", 0.2);
  params->SetReal("BehaviorMobilRuleBased::Politeness", 0.5);
  params->SetReal("BehaviorMobilRuleBased::BSafe", 4.0);

  // Setting Up Map
  auto open_drive_map = MakeXodrMapEndingLaneInParallel();
  auto map_interface = std::make_shared<MapInterface>();
  map_interface->interface_from_opendrive(open_drive_map);

  Polygon car_polygon = CarRectangle();

  Polygon polygon = GenerateGoalRectangle(6, 3);
  std::shared_ptr<Polygon> goal_polygon(
      std::dynamic_pointer_cast<Polygon>(polygon.Translate(
          Point2d(100, -2))));  // < move the goal polygon into the driving
                                // corridor in front of the ego vehicle
  auto goal_definition_ptr =
      std::make_shared<GoalDefinitionPolygon>(*goal_polygon);

  ExecutionModelPtr exec_model(new ExecutionModelInterpolate(params));
  DynamicModelPtr dyn_model(new SingleTrackModel(params));
  BehaviorModelPtr beh_model(new BehaviorMobilRuleBased(params));

  State init_state1(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state1 << 0.0, 30.0, -1.75 - 3.5, 0.0, vel_ego;
  AgentPtr agent1(new Agent(init_state1, beh_model, dyn_model, exec_model,
                            car_polygon, params, goal_definition_ptr,
                            map_interface, bark::geometry::Model3D()));

  // Construct World
  WorldPtr world(new World(params));
  world->AddAgent(agent1);
  world->UpdateAgentRTree();

  WorldPtr current_world_state(world->Clone());
  ObservedWorld observed_world(current_world_state, agent1->GetAgentId());

  const BehaviorModelPtr behavior_model = agent1->GetBehaviorModel();

  auto behavior_mobil =
      std::dynamic_pointer_cast<BehaviorMobilRuleBased>(behavior_model);
  behavior_mobil->SetLaneCorridor(observed_world.GetLaneCorridor());

  LaneChangeDecision decision;
  LaneCorridorPtr lane_corr;
  std::tie(decision, lane_corr) =
      behavior_mobil->CheckIfLaneChangeBeneficial(observed_world);
  EXPECT_EQ(decision, LaneChangeDecision::ChangeLane);
  BARK_EXPECT_TRUE(lane_corr != nullptr);
}

TEST(no_lane_change_to_ending_lane, behavior_mobil) {
  double vel = 5.0;
  auto params = std::make_shared<SetterParams>();
  params->SetReal("BehaviorIDMClassic::DesiredVelocity", vel);
  params->SetBool("BehaviorIDMClassic::BrakeForLaneEnd", true);
  params->SetReal("BehaviorIDMClassic::BrakeForLaneEndEnabledDistance", 60);
  params->SetReal("BehaviorIDMClassic::MaxAcceleration", 1.9);
  params->SetReal("BehaviorMobilRuleBased::AThr", 0.1);
  params->SetReal("BehaviorMobilRuleBased::Politeness", 0.5);
  params->SetReal("BehaviorMobilRuleBased::BSafe", 4.0);

  // Setting Up Map
  auto open_drive_map = MakeXodrMapEndingLaneInParallel();
  auto map_interface = std::make_shared<MapInterface>();
  map_interface->interface_from_opendrive(open_drive_map);

  Polygon car_polygon = CarRectangle();

  Polygon polygon = GenerateGoalRectangle(6, 3);
  std::shared_ptr<Polygon> goal_polygon(
      std::dynamic_pointer_cast<Polygon>(polygon.Translate(
          Point2d(100, -2))));  // < move the goal polygon into the driving
                                // corridor in front of the ego vehicle
  auto goal_definition_ptr =
      std::make_shared<GoalDefinitionPolygon>(*goal_polygon);

  ExecutionModelPtr exec_model(new ExecutionModelInterpolate(params));
  DynamicModelPtr dyn_model(new SingleTrackModel(params));
  BehaviorModelPtr beh_model(new BehaviorMobilRuleBased(params));

  State init_state1(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state1 << 0.0, 30.0, -1.75, 0.0, vel;
  AgentPtr agent1(new Agent(init_state1, beh_model, dyn_model, exec_model,
                            car_polygon, params, goal_definition_ptr,
                            map_interface, bark::geometry::Model3D()));

  // Preceding Agent
  ExecutionModelPtr exec_model2(new ExecutionModelInterpolate(params));
  DynamicModelPtr dyn_model2(new SingleTrackModel(params));
  BehaviorModelPtr beh_model2(new BehaviorConstantAcceleration(params));

  State init_state2(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state2 << 0.0, 60.0, -1.75, 0.0, vel;
  AgentPtr agent2(new Agent(init_state2, beh_model2, dyn_model2, exec_model2,
                            car_polygon, params, goal_definition_ptr,
                            map_interface, bark::geometry::Model3D()));

  // Construct World
  WorldPtr world(new World(params));
  world->AddAgent(agent1);
  world->AddAgent(agent2);
  world->UpdateAgentRTree();

  WorldPtr current_world_state(world->Clone());
  ObservedWorld observed_world(current_world_state, agent1->GetAgentId());

  const BehaviorModelPtr behavior_model = agent1->GetBehaviorModel();

  auto behavior_mobil =
      std::dynamic_pointer_cast<BehaviorMobilRuleBased>(behavior_model);
  behavior_mobil->SetLaneCorridor(observed_world.GetLaneCorridor());

  LaneChangeDecision decision;
  LaneCorridorPtr lane_corr;
  std::tie(decision, lane_corr) =
      behavior_mobil->CheckIfLaneChangeBeneficial(observed_world);
  EXPECT_EQ(decision, LaneChangeDecision::KeepLane);
  BARK_EXPECT_TRUE(lane_corr != nullptr);
}

int main(int argc, char** argv) {
  // FLAGS_v=2;
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}