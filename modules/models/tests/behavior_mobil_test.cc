// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <Eigen/Core>
#include "gtest/gtest.h"

#include "modules/commons/params/setter_params.hpp"
#include "modules/geometry/polygon.hpp"
#include "modules/geometry/standard_shapes.hpp"
#include "modules/models/behavior/mobil/mobil.hpp"
#include "modules/models/dynamic/single_track.hpp"
#include "modules/models/execution/interpolation/interpolate.hpp"
#include "modules/world/goal_definition/goal_definition_polygon.hpp"
#include "modules/world/objects/agent.hpp"
#include "modules/world/observed_world.hpp"
#include "modules/world/tests/make_test_xodr_map.hpp"

using namespace modules::models::dynamic;
using namespace modules::models::execution;
using namespace modules::commons;
using namespace modules::models::behavior;
using namespace modules::world::map;
using namespace modules::models::dynamic;

using modules::geometry::Model3D;
using modules::geometry::Point2d;
using modules::geometry::Polygon;
using modules::geometry::Pose;
using modules::geometry::standard_shapes::CarRectangle;
using modules::world::ObservedWorld;
using modules::world::World;
using modules::world::WorldPtr;
using modules::world::goal_definition::GoalDefinitionPolygon;
using modules::world::objects::Agent;
using modules::world::objects::AgentPtr;
using modules::world::tests::MakeXodrMapOneRoadTwoLanes;

TEST(slower_preceding_agent, behavior_mobil) {
  // Setting Up Map
  auto open_drive_map = MakeXodrMapOneRoadTwoLanes();
  auto map_interface = std::make_shared<MapInterface>();
  map_interface->interface_from_opendrive(open_drive_map);

  auto params = std::make_shared<SetterParams>();
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

  // Ego Agent
  ExecutionModelPtr exec_model(new ExecutionModelInterpolate(params));
  DynamicModelPtr dyn_model(new SingleTrackModel(params));
  BehaviorModelPtr beh_model(new BehaviorMobil(params));

  State init_state1(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state1 << 0.0, 3.0, -1.75, 0.0, 5.0;
  AgentPtr agent1(new Agent(init_state1, beh_model, dyn_model, exec_model,
                            car_polygon, params, goal_definition_ptr,
                            map_interface, modules::geometry::Model3D()));

  // Preceding Agent
  ExecutionModelPtr exec_model2(new ExecutionModelInterpolate(params));
  DynamicModelPtr dyn_model2(new SingleTrackModel(params));
  BehaviorModelPtr beh_model2(new BehaviorMobil(params));

  State init_state2(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state2 << 0.0, 15.0, -1.75, 0.0, 2.0;
  AgentPtr agent2(new Agent(init_state2, beh_model2, dyn_model2, exec_model2,
                            car_polygon, params, goal_definition_ptr,
                            map_interface, modules::geometry::Model3D()));

  // Construct World
  WorldPtr world(new World(params));
  world->AddAgent(agent1);
  world->AddAgent(agent2);
  world->UpdateAgentRTree();

  WorldPtr current_world_state(world->Clone());
  ObservedWorld observed_world(current_world_state, agent1->GetAgentId());

  const BehaviorModelPtr behavior_model = agent1->GetBehaviorModel();

  auto behavior_mobil =
      std::dynamic_pointer_cast<BehaviorMobil>(behavior_model);

  LaneChangeDecision decision;
  LaneCorridorPtr lane_corr;
  std::tie(decision, lane_corr) =
      behavior_mobil->CheckIfLaneChangeBeneficial(observed_world);
  EXPECT_EQ(decision, LaneChangeDecision::ChangeRight);
  BARK_EXPECT_TRUE(lane_corr != nullptr);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}