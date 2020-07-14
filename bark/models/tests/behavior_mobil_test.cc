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
#include "bark/geometry/polygon.hpp"
#include "bark/geometry/standard_shapes.hpp"
#include "bark/models/behavior/rule_based/mobil.hpp"
#include "bark/models/behavior/rule_based/mobil_behavior.hpp"
#include "bark/models/behavior/constant_velocity/constant_velocity.hpp"
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
using bark::world::ObservedWorld;
using bark::world::World;
using bark::world::WorldPtr;
using bark::world::goal_definition::GoalDefinitionPolygon;
using bark::world::objects::Agent;
using bark::world::objects::AgentPtr;
using bark::world::tests::MakeXodrMapOneRoadTwoLanes;

ObservedWorld make_observed_world_three_agents(double ego_vel, ParamsPtr params) {

  // Setting Up Map
  auto open_drive_map = MakeXodrMapOneRoadTwoLanes();
  auto map_interface = std::make_shared<MapInterface>();
  map_interface->interface_from_opendrive(open_drive_map);

  Polygon car_polygon = CarRectangle();

  Polygon polygon(
      Pose(1, 1, 0),
      std::vector<Point2d>{Point2d(0, 0), Point2d(0, 2), Point2d(2, 2),
                           Point2d(2, 0), Point2d(0, 0)});
  std::shared_ptr<Polygon> goal_polygon(
      std::dynamic_pointer_cast<Polygon>(polygon.Translate(
          Point2d(50, -2))));  // < move the goal polygon into the driving
                               // corridor in front of the ego vehicle
  auto goal_definition_ptr =
      std::make_shared<GoalDefinitionPolygon>(*goal_polygon);

  ExecutionModelPtr exec_model(new ExecutionModelInterpolate(params));
  DynamicModelPtr dyn_model(new SingleTrackModel(params));
  BehaviorModelPtr beh_model(new BehaviorMobilRuleBased(params));

  State init_state1(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state1 << 0.0, 13.0, -1.75, 0.0, ego_vel;
  AgentPtr agent1(new Agent(init_state1, beh_model, dyn_model, exec_model,
                            car_polygon, params, goal_definition_ptr,
                            map_interface, bark::geometry::Model3D()));

  // Preceding Agent
  ExecutionModelPtr exec_model2(new ExecutionModelInterpolate(params));
  DynamicModelPtr dyn_model2(new SingleTrackModel(params));
  BehaviorModelPtr beh_model2(new BehaviorConstantVelocity(params));

  State init_state2(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state2 << 0.0, 40.0, -1.75, 0.0, 5;
  AgentPtr agent2(new Agent(init_state2, beh_model2, dyn_model2, exec_model2,
                            car_polygon, params, goal_definition_ptr,
                            map_interface, bark::geometry::Model3D()));

  // Preceding Agent
  ExecutionModelPtr exec_model3(new ExecutionModelInterpolate(params));
  DynamicModelPtr dyn_model3(new SingleTrackModel(params));
  BehaviorModelPtr beh_model3(new BehaviorConstantVelocity(params));

  State init_state3(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state3 << 0.0, 3.0, -1.75-3.5, 0.0, 5;
  AgentPtr agent3(new Agent(init_state3, beh_model3, dyn_model3, exec_model3,
                            car_polygon, params, goal_definition_ptr,
                            map_interface, bark::geometry::Model3D()));

  // Construct World
  WorldPtr world(new World(params));
  world->AddAgent(agent1);
  world->AddAgent(agent2);
  world->AddAgent(agent3);
  world->UpdateAgentRTree();

  WorldPtr current_world_state(world->Clone());
  ObservedWorld observed_world(current_world_state, agent1->GetAgentId());

  const BehaviorModelPtr behavior_model = agent1->GetBehaviorModel();

  auto behavior_mobil =
      std::dynamic_pointer_cast<BehaviorMobilRuleBased>(behavior_model);
  behavior_mobil->SetLaneCorridor(observed_world.GetLaneCorridor());
  
  return observed_world;
}


TEST(safety_not_met, behavior_mobil) {

  auto params = std::make_shared<SetterParams>();

  double ego_vel = 5.0; // set for all?
  params->SetReal("BehaviorIDMClassic::DesiredVelocity", ego_vel);
  params->SetReal("BehaviorMobilRuleBased::AThr", 0.2);
  params->SetReal("BehaviorMobilRuleBased::Politeness", 0.0);
  params->SetReal("BehaviorMobilRuleBased::BSafe", 4.0);

  ObservedWorld observed_world = make_observed_world_three_agents(ego_vel, params);

  const BehaviorModelPtr behavior_model = observed_world.GetEgoAgent()->GetBehaviorModel();
  auto behavior_mobil = std::dynamic_pointer_cast<BehaviorMobilRuleBased>(behavior_model);
  behavior_mobil->SetLaneCorridor(observed_world.GetLaneCorridor());

  LaneChangeDecision decision;
  LaneCorridorPtr lane_corr;
  std::tie(decision, lane_corr) =
      behavior_mobil->CheckIfLaneChangeBeneficial(observed_world);
  EXPECT_EQ(decision, LaneChangeDecision::KeepLane);
  BARK_EXPECT_TRUE(lane_corr != nullptr);
}

TEST(polite_incentive_met_safety_met, behavior_mobil) {

  auto params = std::make_shared<SetterParams>();

  double ego_vel = 5.0; // set for all?
  params->SetReal("BehaviorIDMClassic::DesiredVelocity", ego_vel);
  params->SetReal("BehaviorMobilRuleBased::AThr", 0.2);
  params->SetReal("BehaviorMobilRuleBased::Politeness", 0.0);
  params->SetReal("BehaviorMobilRuleBased::BSafe", 6.0);

  ObservedWorld observed_world = make_observed_world_three_agents(ego_vel, params);

  const BehaviorModelPtr behavior_model = observed_world.GetEgoAgent()->GetBehaviorModel();
  auto behavior_mobil = std::dynamic_pointer_cast<BehaviorMobilRuleBased>(behavior_model);
  behavior_mobil->SetLaneCorridor(observed_world.GetLaneCorridor());

  LaneChangeDecision decision;
  LaneCorridorPtr lane_corr;
  std::tie(decision, lane_corr) =
      behavior_mobil->CheckIfLaneChangeBeneficial(observed_world);
  EXPECT_EQ(decision, LaneChangeDecision::ChangeLane);
  BARK_EXPECT_TRUE(lane_corr != nullptr);
}

TEST(impolite_incentive_not_met_safety_met, behavior_mobil) {

  auto params = std::make_shared<SetterParams>();

  double ego_vel = 5.0; // set for all?
  params->SetReal("BehaviorIDMClassic::DesiredVelocity", ego_vel);
  params->SetReal("BehaviorMobilRuleBased::AThr", 0.2);
  params->SetReal("BehaviorMobilRuleBased::Politeness", 1.0);
  params->SetReal("BehaviorMobilRuleBased::BSafe", 6.0);

  ObservedWorld observed_world = make_observed_world_three_agents(ego_vel, params);

  const BehaviorModelPtr behavior_model = observed_world.GetEgoAgent()->GetBehaviorModel();
  auto behavior_mobil = std::dynamic_pointer_cast<BehaviorMobilRuleBased>(behavior_model);
  behavior_mobil->SetLaneCorridor(observed_world.GetLaneCorridor());

  LaneChangeDecision decision;
  LaneCorridorPtr lane_corr;
  std::tie(decision, lane_corr) =
      behavior_mobil->CheckIfLaneChangeBeneficial(observed_world);
  EXPECT_EQ(decision, LaneChangeDecision::KeepLane);
  BARK_EXPECT_TRUE(lane_corr != nullptr);
}

TEST(impolite_incentive_met_safety_met, behavior_mobil) {

  auto params = std::make_shared<SetterParams>();

  double ego_vel = 5.0; // set for all?
  params->SetReal("BehaviorIDMClassic::DesiredVelocity", ego_vel);
  params->SetReal("BehaviorMobilRuleBased::AThr", -5.0); // HACK
  params->SetReal("BehaviorMobilRuleBased::Politeness", 1.0);
  params->SetReal("BehaviorMobilRuleBased::BSafe", 6.0);

  ObservedWorld observed_world = make_observed_world_three_agents(ego_vel, params);

  const BehaviorModelPtr behavior_model = observed_world.GetEgoAgent()->GetBehaviorModel();
  auto behavior_mobil = std::dynamic_pointer_cast<BehaviorMobilRuleBased>(behavior_model);
  behavior_mobil->SetLaneCorridor(observed_world.GetLaneCorridor());

  LaneChangeDecision decision;
  LaneCorridorPtr lane_corr;
  std::tie(decision, lane_corr) =
      behavior_mobil->CheckIfLaneChangeBeneficial(observed_world);
  EXPECT_EQ(decision, LaneChangeDecision::ChangeLane);
  BARK_EXPECT_TRUE(lane_corr != nullptr);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}