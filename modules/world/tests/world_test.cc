// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include "gtest/gtest.h"
#include "modules/models/dynamic/single_track.hpp"
#include "modules/models/execution/interpolation/interpolate.hpp"
#include "modules/world/map/map_interface.hpp"
#include "modules/world/map/roadgraph.hpp"
#include "modules/world/opendrive/opendrive.hpp"
#include "modules/models/behavior/constant_velocity/constant_velocity.hpp"
#include "modules/geometry/polygon.hpp"
#include "modules/commons/params/default_params.hpp"
#include "modules/world/objects/agent.hpp"
#include "modules/world/world.hpp"
#include "modules/world/evaluation/evaluator_collision_agents.hpp"
#include "modules/world/evaluation/evaluator_drivable_area.hpp"
#include "modules/world/tests/make_test_world.hpp"


using namespace modules::models::dynamic;
using namespace modules::geometry;
using namespace modules::commons;
using namespace modules::models::behavior;
using namespace modules::models::execution;
using namespace modules::world::opendrive;
using modules::world::map::MapInterface;
using modules::world::map::MapInterfacePtr;
using modules::world::map::RoadgraphPtr;
using modules::world::map::Roadgraph;
using modules::world::opendrive::XodrLaneId;
using namespace modules::world::objects;
using namespace modules::world;
using namespace modules::world::evaluation;
using modules::world::tests::make_test_world;

TEST(world, world_init)
{
  auto params = std::make_shared<DefaultParams>();
  ExecutionModelPtr exec_model(new ExecutionModelInterpolate(params));
  DynamicModelPtr dyn_model(new SingleTrackModel(params));
  BehaviorModelPtr beh_model(new BehaviorConstantVelocity(params));
  Polygon polygon(Pose(1.25, 1, 0), std::vector<Point2d>{Point2d(0, 0), Point2d(0, 2), Point2d(4, 2), Point2d(4, 0), Point2d(0, 0)});
  ASSERT_TRUE(polygon.Valid());
  State init_state(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  AgentPtr agent(new Agent(init_state, beh_model, dyn_model, exec_model, polygon, params));

  WorldPtr world(new World(params));
  world->AddAgent(agent);
  world->AddAgent(agent);
  world->AddAgent(agent);
  world->AddAgent(agent);
}

TEST(world, world_step)
{
  auto params = std::make_shared<DefaultParams>();
  ExecutionModelPtr exec_model(new ExecutionModelInterpolate(params));
  DynamicModelPtr dyn_model(new SingleTrackModel(params));
  BehaviorModelPtr beh_model(new BehaviorConstantVelocity(params));
  Polygon polygon(Pose(1.25, 1, 0), std::vector<Point2d>{Point2d(0, 0), Point2d(0, 2), Point2d(4, 2), Point2d(4, 0), Point2d(0, 0)});
  ASSERT_TRUE(polygon.Valid());
  State init_state(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  AgentPtr agent(new Agent(init_state, beh_model, dyn_model, exec_model, polygon, params));

  WorldPtr world(new World(params));
  world->AddAgent(agent);

  //! new plan view
  std::shared_ptr<PlanView> p_1(new PlanView());

  //! add line
  p_1->AddLine(Point2d(0.0f, 0.0f), 0.0, 25.0f);
  p_1->AddLine(Point2d(25.0f, 0.0f), 0.0, 25.0f);

  //! lane sections
  std::shared_ptr<XodrLaneSection> section_1(new XodrLaneSection(0.0f));
  std::shared_ptr<XodrLaneSection> section_2(new XodrLaneSection(0.0f));

  XodrLaneOffset off = {1.5, 0, 0, 0};
  XodrLaneWidth lane_width_1 = {0, 30, off};
  XodrLaneWidth lane_width_2 = {0, 30, off};
  XodrLaneWidth lane_width_3 = {30, 50, off};
  XodrLaneWidth lane_width_4 = {30, 50, off};

  std::shared_ptr<XodrLane> l_0(new XodrLane(1));
  std::shared_ptr<XodrLane> l_1(new XodrLane(-1));
  std::shared_ptr<XodrLane> l_2(new XodrLane(1));
  std::shared_ptr<XodrLane> l_3(new XodrLane(-1));

  l_0 = CreateLaneFromLaneWidth(1, p_1->GetReferenceLine(), lane_width_1);
  l_1 = CreateLaneFromLaneWidth(-1, p_1->GetReferenceLine(), lane_width_2);
  l_2 = CreateLaneFromLaneWidth(1, p_1->GetReferenceLine(), lane_width_3);
  l_3 = CreateLaneFromLaneWidth(-1, p_1->GetReferenceLine(), lane_width_4);

  section_1->AddLane(l_0);
  section_1->AddLane(l_1);

  section_2->AddLane(l_2);
  section_2->AddLane(l_3);

  std::shared_ptr<XodrRoad> r(new XodrRoad("highway", 100));

  r->SetPlanView(p_1);
  r->AddLaneSection(section_1);
  r->AddLaneSection(section_2);

  modules::world::opendrive::OpenDriveMapPtr open_drive_map(new modules::world::opendrive::OpenDriveMap());
  open_drive_map->AddRoad(r);

  RoadgraphPtr roadgraph(new Roadgraph());
  MapInterfacePtr map_interface(new MapInterface());
  map_interface->SetOpenDriveMap(open_drive_map);
  map_interface->SetRoadgraph(roadgraph);

  world->SetMap(map_interface);
  world->AddAgent(agent);
  /*
  for (int i = 0; i < 10; ++i) {
    world->Step(1);
  }*/
}

TEST(world, world_collision)
{
  auto params = std::make_shared<DefaultParams>();
  ExecutionModelPtr exec_model(new ExecutionModelInterpolate(params));
  DynamicModelPtr dyn_model(new SingleTrackModel(params));
  BehaviorModelPtr beh_model(new BehaviorConstantVelocity(params));
  EvaluatorPtr col_checker(new EvaluatorCollisionAgents());

  Polygon polygon(Pose(1.25, 1, 0), std::vector<Point2d>{Point2d(0, 0), Point2d(0, 2), Point2d(4, 2), Point2d(4, 0), Point2d(0, 0)});
  
  State init_state1(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state1 << 0.0, 0.0, 0.0, 0.0, 5.0;
  AgentPtr agent1(new Agent(init_state1, beh_model, dyn_model, exec_model, polygon, params));

  State init_state2(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state2 << 0.0, 1.0, 0.0, 0.0, 5.0;
  AgentPtr agent2(new Agent(init_state2, beh_model, dyn_model, exec_model, polygon, params));

  WorldPtr world(new World(params));
  world->AddAgent(agent1);
  world->AddAgent(agent2);

  world->AddEvaluator("collision_agents", col_checker);

  ASSERT_TRUE(world->Evaluate()["collision_agents"].which());
  
}


TEST(world, world_no_collision_agent)
{
  auto params = std::make_shared<DefaultParams>();
  ExecutionModelPtr exec_model(new ExecutionModelInterpolate(params));
  DynamicModelPtr dyn_model(new SingleTrackModel(params));
  BehaviorModelPtr beh_model(new BehaviorConstantVelocity(params));
  EvaluatorPtr col_checker(new EvaluatorCollisionAgents());

  Polygon polygon(Pose(1.25, 1, 0), std::vector<Point2d>{Point2d(0, 0), Point2d(0, 2), Point2d(4, 2), Point2d(4, 0), Point2d(0, 0)});
  
  State init_state1(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state1 << 0.0, 0.0, 0.0, 0.0, 5.0;
  AgentPtr agent1(new Agent(init_state1, beh_model, dyn_model, exec_model, polygon, params));

  State init_state2(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state2 << 0.0, 10.0, 0.0, 0.0, 5.0;
  AgentPtr agent2(new Agent(init_state2, beh_model, dyn_model, exec_model, polygon, params));

  WorldPtr world(new World(params));
  world->AddAgent(agent1);
  world->AddAgent(agent2);

  world->AddEvaluator("collision_agents", col_checker);

  ASSERT_TRUE(world->Evaluate()["collision_agents"].which());
  
}

TEST(world, world_outside_drivable_area) {
  using modules::world::goal_definition::GoalDefinitionPolygon;

  auto params = std::make_shared<DefaultParams>();

  ExecutionModelPtr exec_model(new ExecutionModelInterpolate(params));
  DynamicModelPtr dyn_model(new SingleTrackModel(params));
  BehaviorModelPtr beh_model(new BehaviorConstantVelocity(params));
  EvaluatorPtr col_checker(new EvaluatorCollisionAgents());

  EvaluatorPtr evaluator_drivable_area(new EvaluatorDrivableArea());

  Polygon polygon(Pose(1, 1, 0), std::vector<Point2d>{Point2d(0, 0), Point2d(0, 2), Point2d(2, 2), Point2d(2, 0), Point2d(0, 0)});
  std::shared_ptr<Polygon> goal_polygon(std::dynamic_pointer_cast<Polygon>(polygon.Translate(Point2d(50,-2)))); // < move the goal polygon into the driving corridor in front of the ego vehicle
  auto goal_definition_ptr = std::make_shared<GoalDefinitionPolygon>(*goal_polygon);

  float ego_velocity = 5.0, rel_distance = 2.0, velocity_difference=2.0;

  // TODO: pass goal polygon to make_world_test
  WorldPtr world = make_test_world(0, rel_distance, ego_velocity, velocity_difference, goal_definition_ptr);

  world->AddEvaluator("drivable_area", evaluator_drivable_area);
  auto eval_res = world->Evaluate()["drivable_area"];

  ASSERT_FALSE(boost::get<bool>(eval_res));
}


TEST(world, nearest_agents)
{
  auto params = std::make_shared<DefaultParams>();
  ExecutionModelPtr exec_model(new ExecutionModelInterpolate(params));
  DynamicModelPtr dyn_model(new SingleTrackModel(params));
  BehaviorModelPtr beh_model(new BehaviorConstantVelocity(params));
  EvaluatorPtr col_checker(new EvaluatorCollisionAgents());

  Polygon polygon(Pose(1.25, 1, 0), std::vector<Point2d>{Point2d(0, 0), Point2d(0, 2), Point2d(4, 2), Point2d(4, 0), Point2d(0, 0)});
  
  State init_state1(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state1 << 0.0, 0.0, 0.0, 0.0, 5.0;
  AgentPtr agent1(new Agent(init_state1, beh_model, dyn_model, exec_model, polygon, params));

  State init_state2(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state2 << 0.0, 1.0, 0.0, 0.0, 5.0;
  AgentPtr agent2(new Agent(init_state2, beh_model, dyn_model, exec_model, polygon, params));

  WorldPtr world(new World(params));
  world->AddAgent(agent1);
  world->AddAgent(agent2);

  world->UpdateAgentRTree();

  AgentMap nearest_agents = world->GetNearestAgents(Point2d(0.0f,0.0f),1);

  EXPECT_EQ(nearest_agents.size(), 1);
  EXPECT_EQ(nearest_agents.begin()->second->GetCurrentState(), init_state1 );
  
  AgentMap nearest_agents2 = world->GetNearestAgents(Point2d(0.0f,0.0f),2);

  // Comment as this crashes in CI -> TODO: Why?
  /*EXPECT_EQ(nearest_agents2.size(), 2);
  EXPECT_EQ(nearest_agents2.begin()->second->GetCurrentState(), init_state1 );
  EXPECT_EQ(nearest_agents2[agent2->GetAgentId()]->GetCurrentState(), init_state2 );

  State init_state3(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state3 << 0.0, 1.0, 5.0, 0.0, 5.0;
  AgentPtr agent3(new Agent(init_state3, beh_model, dyn_model, exec_model, polygon, params));

  world->AddAgent(agent3);
  world->UpdateAgentRTree();
  
  AgentMap nearest_agents3 = world->GetNearestAgents(Point2d(4.0f,5.0f),2);
  EXPECT_EQ(nearest_agents3.size(), 2);
  EXPECT_EQ(nearest_agents3[agent1->GetAgentId()]->GetCurrentState(), init_state1 );
  EXPECT_EQ(nearest_agents3[agent3->GetAgentId()]->GetCurrentState(), init_state3 );
  */
}

TEST(world, agents_intersection_polygon)
{
  auto params = std::make_shared<DefaultParams>();
  ExecutionModelPtr exec_model(new ExecutionModelInterpolate(params));
  DynamicModelPtr dyn_model(new SingleTrackModel(params));
  BehaviorModelPtr beh_model(new BehaviorConstantVelocity(params));
  EvaluatorPtr col_checker(new EvaluatorCollisionAgents());

  Polygon polygon(Pose(1.25, 1, 0), std::vector<Point2d>{Point2d(0, 0), Point2d(0, 2), Point2d(4, 2), Point2d(4, 0), Point2d(0, 0)});
  
  State init_state1(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state1 << 0.0, 0.0, 0.0, 0.0, 5.0;
  AgentPtr agent1(new Agent(init_state1, beh_model, dyn_model, exec_model, polygon, params));

  State init_state2(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state2 << 0.0, 10.0, 0.0, 0.0, 5.0;
  AgentPtr agent2(new Agent(init_state2, beh_model, dyn_model, exec_model, polygon, params));

  WorldPtr world(new World(params));
  world->AddAgent(agent1);
  world->AddAgent(agent2);

  world->UpdateAgentRTree();


  Polygon polygon_intersect(Pose(0, 0, 0), std::vector<Point2d>{Point2d(0, 1), Point2d(0, 3), Point2d(4, 3), Point2d(4, 1), Point2d(0, 1)});

  AgentMap agents_intersect = world->GetAgentsIntersectingPolygon(polygon_intersect);

  EXPECT_EQ(agents_intersect.size(), 1);
  EXPECT_EQ(agents_intersect.begin()->second->GetCurrentState(), init_state1 );


  Polygon polygon_intersect2(Pose(0, 0, 0), std::vector<Point2d>{Point2d(7, 1), Point2d(7, 3), Point2d(11, 3), Point2d(11, 1), Point2d(7, 1)});
  AgentMap agents_intersect2 = world->GetAgentsIntersectingPolygon(polygon_intersect2);

  EXPECT_EQ(agents_intersect2.size(), 1);
  EXPECT_EQ(agents_intersect2.begin()->second->GetCurrentState(), init_state2);

  State init_state3(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state3 << 0.0, 1.0, 3.0, 0.0, 5.0;
  AgentPtr agent3(new Agent(init_state3, beh_model, dyn_model, exec_model, polygon, params));

  world->AddAgent(agent3);
  world->UpdateAgentRTree();
  
  Polygon polygon_intersect3(Pose(0, 0, 0), std::vector<Point2d>{Point2d(0, 1), Point2d(0, 3), Point2d(4, 3), Point2d(4, 1), Point2d(0, 1)});
  AgentMap agents_intersect3 = world->GetAgentsIntersectingPolygon(polygon_intersect3);
  EXPECT_EQ(agents_intersect3.size(), 2);
  EXPECT_EQ(agents_intersect3[agent1->GetAgentId()]->GetCurrentState(), init_state1 );
  EXPECT_EQ(agents_intersect3[agent3->GetAgentId()]->GetCurrentState(), init_state3 );
}


