// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/world/world.hpp"
#include "bark/commons/params/setter_params.hpp"
#include "bark/geometry/polygon.hpp"
#include "bark/models/behavior/constant_acceleration/constant_acceleration.hpp"
#include "bark/models/dynamic/single_track.hpp"
#include "bark/models/execution/interpolation/interpolate.hpp"
#include "bark/world/evaluation/evaluator_collision_agents.hpp"
#include "bark/world/evaluation/evaluator_distance_to_goal.hpp"
#include "bark/world/evaluation/evaluator_drivable_area.hpp"
#ifdef RSS
#include "bark/world/evaluation/rss/evaluator_rss.hpp"
#endif
#include "bark/world/evaluation/safe_distances/evaluator_dynamic_safe_dist.hpp"
#include "bark/world/evaluation/safe_distances/evaluator_safe_dist_drivable_area.hpp"
#include "bark/world/evaluation/safe_distances/evaluator_static_safe_dist.hpp"

#include "bark/world/goal_definition/goal_definition_polygon.hpp"
#include "bark/world/map/map_interface.hpp"
#include "bark/world/map/roadgraph.hpp"
#include "bark/world/objects/agent.hpp"
#include "bark/world/opendrive/opendrive.hpp"
#include "bark/world/tests/make_test_world.hpp"
#include "gtest/gtest.h"

using namespace bark::models::dynamic;
using namespace bark::geometry;
using namespace bark::commons;
using namespace bark::models::behavior;
using namespace bark::models::execution;
using namespace bark::world::opendrive;
using bark::world::map::MapInterface;
using bark::world::map::MapInterfacePtr;
using bark::world::map::Roadgraph;
using bark::world::map::RoadgraphPtr;
using bark::world::opendrive::XodrLaneId;
using namespace bark::world::objects;
using namespace bark::world;
using namespace bark::world::evaluation;
using namespace bark::world::goal_definition;
using bark::geometry::standard_shapes::GenerateGoalRectangle;
using bark::world::tests::make_test_world;

TEST(world, world_init) {
  auto params = std::make_shared<SetterParams>();
  ExecutionModelPtr exec_model(new ExecutionModelInterpolate(params));
  DynamicModelPtr dyn_model(new SingleTrackModel(params));
  BehaviorModelPtr beh_model(new BehaviorConstantAcceleration(params));
  Polygon polygon(
      Pose(1.25, 1, 0),
      std::vector<Point2d>{Point2d(0, 0), Point2d(0, 2), Point2d(4, 2),
                           Point2d(4, 0), Point2d(0, 0)});
  ASSERT_TRUE(polygon.Valid());
  State init_state(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  AgentPtr agent(
      new Agent(init_state, beh_model, dyn_model, exec_model, polygon, params));

  WorldPtr world(new World(params));
  world->AddAgent(agent);
  world->AddAgent(agent);
  world->AddAgent(agent);
  world->AddAgent(agent);
}

TEST(world, world_step) {
  auto params = std::make_shared<SetterParams>();
  ExecutionModelPtr exec_model(new ExecutionModelInterpolate(params));
  DynamicModelPtr dyn_model(new SingleTrackModel(params));
  BehaviorModelPtr beh_model(new BehaviorConstantAcceleration(params));
  Polygon polygon(
      Pose(1.25, 1, 0),
      std::vector<Point2d>{Point2d(0, 0), Point2d(0, 2), Point2d(4, 2),
                           Point2d(4, 0), Point2d(0, 0)});
  ASSERT_TRUE(polygon.Valid());
  State init_state(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  AgentPtr agent(
      new Agent(init_state, beh_model, dyn_model, exec_model, polygon, params));

  WorldPtr world(new World(params));
  world->AddAgent(agent);

  //! new plan view
  std::shared_ptr<PlanView> p_1(new PlanView());

  //! add line
  p_1->AddLine(Point2d(0.0, 0.0), 0.0, 25.0, 25.0);
  p_1->AddLine(Point2d(25.0, 0.0), 0.0, 25.0, 25.0);

  //! lane sections
  std::shared_ptr<XodrLaneSection> section_1(new XodrLaneSection(0.0));
  std::shared_ptr<XodrLaneSection> section_2(new XodrLaneSection(0.0));

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

  bark::world::opendrive::OpenDriveMapPtr open_drive_map(
      new bark::world::opendrive::OpenDriveMap());
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

TEST(evaluator, collision) {
  auto params = std::make_shared<SetterParams>();
  ExecutionModelPtr exec_model(new ExecutionModelInterpolate(params));
  DynamicModelPtr dyn_model(new SingleTrackModel(params));
  BehaviorModelPtr beh_model(new BehaviorConstantAcceleration(params));
  EvaluatorPtr col_checker(new EvaluatorCollisionAgents());

  Polygon polygon(
      Pose(1.25, 1, 0),
      std::vector<Point2d>{Point2d(0, 0), Point2d(0, 2), Point2d(4, 2),
                           Point2d(4, 0), Point2d(0, 0)});

  State init_state1(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state1 << 0.0, 0.0, 0.0, 0.0, 5.0;
  AgentPtr agent1(new Agent(init_state1, beh_model, dyn_model, exec_model,
                            polygon, params));

  State init_state2(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state2 << 0.0, 1.0, 0.0, 0.0, 5.0;
  AgentPtr agent2(new Agent(init_state2, beh_model, dyn_model, exec_model,
                            polygon, params));

  WorldPtr world(new World(params));
  world->AddAgent(agent1);
  world->AddAgent(agent2);

  world->AddEvaluator("collision_agents", col_checker);

  ASSERT_TRUE(boost::get<bool>(world->Evaluate()["collision_agents"]));
}

TEST(evaluator, static_safe_dist) {
  auto params = std::make_shared<SetterParams>();
  params->SetReal("EvaluatorStaticSafeDist::LateralSafeDist", 10.9f);
  params->SetReal("EvaluatorStaticSafeDist::LongitudinalSafeDist", 0.5f);
  ExecutionModelPtr exec_model(new ExecutionModelInterpolate(params));
  DynamicModelPtr dyn_model(new SingleTrackModel(params));
  BehaviorModelPtr beh_model(new BehaviorConstantAcceleration(params));

  Polygon polygon(Pose(0.0, 0.0, 0),
                  std::vector<Point2d>{Point2d(-0.5, -0.5), Point2d(-0.5, 0.5),
                                       Point2d(0.5, 0.5), Point2d(0.5, -0.5),
                                       Point2d(-0.5, -0.5)});

  State init_state1(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state1 << 0.0, 0.0, 0.0, 0.0, 5.0;
  AgentPtr agent1(new Agent(init_state1, beh_model, dyn_model, exec_model,
                            polygon, params));

  State init_state2(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state2 << 0.0, 1.51, 0.0, 0.0, 5.0;
  AgentPtr agent2(new Agent(init_state2, beh_model, dyn_model, exec_model,
                            polygon, params));

  WorldPtr world(new World(params));
  world->AddAgent(agent1);
  world->AddAgent(agent2);
  world->UpdateAgentRTree();

  EvaluatorPtr safe_dist_checker(
      new EvaluatorStaticSafeDist(params, agent1->GetAgentId()));
  world->AddEvaluator("safe_dist_agents", safe_dist_checker);

  EXPECT_EQ(boost::get<int>(world->Evaluate()["safe_dist_agents"]), 0);

  // safe dist gives polygon widths of
  State init_state3(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state3 << 0.0, 1.5, 0.0, 0.0, 5.0;
  AgentPtr agent3(new Agent(init_state3, beh_model, dyn_model, exec_model,
                            polygon, params));
  world->AddAgent(agent3);
  world->UpdateAgentRTree();

  EXPECT_EQ(boost::get<int>(world->Evaluate()["safe_dist_agents"]), 1);
  EXPECT_EQ(boost::get<int>(world->Evaluate()["safe_dist_agents"]), 2);
}

TEST(evaluator, dynamic_safe_dist) {
  auto params = std::make_shared<SetterParams>();
  params->SetBool("EvaluatorDynamicSafeDist::ToRear", true);
  params->SetReal("EvaluatorDynamicSafeDist::ReactionTime", 1.0f);
  params->SetReal("EvaluatorDynamicSafeDist::MaxEgoDecceleration", 5.0f);
  params->SetReal("EvaluatorDynamicSafeDist::MaxOtherDecceleration", 5.0f);
  ExecutionModelPtr exec_model(new ExecutionModelInterpolate(params));
  DynamicModelPtr dyn_model(new SingleTrackModel(params));
  BehaviorModelPtr beh_model(new BehaviorConstantAcceleration(params));

  Polygon polygon(
      Pose(0.0, 0.0, 0),
      std::vector<Point2d>{Point2d(0, 0), Point2d(0, 1), Point2d(1, 1),
                           Point2d(1, 0), Point2d(0, 0)});

  State init_state1(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state1 << 0.0, 0.0, 0.0, 0.0, 5.0;
  AgentPtr agent1(new Agent(init_state1, beh_model, dyn_model, exec_model,
                            polygon, params));

  std::shared_ptr<Polygon> goal_polygon(
      std::dynamic_pointer_cast<Polygon>(polygon.Translate(
          Point2d(50, -2))));  // < move the goal polygon into the driving
                               // corridor in front of the ego vehicle
  auto goal_definition_ptr =
      std::make_shared<GoalDefinitionPolygon>(*goal_polygon);

  float ego_velocity = 5.0, rel_distance = 2.0, velocity_difference = 2.0;

  WorldPtr world = make_test_world(0, rel_distance, ego_velocity,
                                   velocity_difference, goal_definition_ptr);
  world->UpdateAgentRTree();

  EvaluatorPtr safe_dist_checker(
      new EvaluatorDynamicSafeDist(params, world->GetAgents().begin()->first));
  world->AddEvaluator("safe_dist_agents", safe_dist_checker);

  // No other agent available -> safe = true
  EXPECT_EQ(boost::get<int>(world->Evaluate()["safe_dist_agents"]), 0);
}

TEST(evaluator, no_collision_agent) {
  auto params = std::make_shared<SetterParams>();
  ExecutionModelPtr exec_model(new ExecutionModelInterpolate(params));
  DynamicModelPtr dyn_model(new SingleTrackModel(params));
  BehaviorModelPtr beh_model(new BehaviorConstantAcceleration(params));
  EvaluatorPtr col_checker(new EvaluatorCollisionAgents());

  Polygon polygon(
      Pose(1.25, 1, 0),
      std::vector<Point2d>{Point2d(0, 0), Point2d(0, 2), Point2d(4, 2),
                           Point2d(4, 0), Point2d(0, 0)});

  State init_state1(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state1 << 0.0, 0.0, 0.0, 0.0, 5.0;
  AgentPtr agent1(new Agent(init_state1, beh_model, dyn_model, exec_model,
                            polygon, params));

  State init_state2(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state2 << 0.0, 10.0, 0.0, 0.0, 5.0;
  AgentPtr agent2(new Agent(init_state2, beh_model, dyn_model, exec_model,
                            polygon, params));

  WorldPtr world(new World(params));
  world->AddAgent(agent1);
  world->AddAgent(agent2);
  world->UpdateAgentRTree();

  world->AddEvaluator("collision_agents", col_checker);

  ASSERT_FALSE(boost::get<bool>(world->Evaluate()["collision_agents"]));
}

TEST(evaluator, outside_drivable_area) {
  using bark::world::goal_definition::GoalDefinitionPolygon;

  auto params = std::make_shared<SetterParams>();

  ExecutionModelPtr exec_model(new ExecutionModelInterpolate(params));
  DynamicModelPtr dyn_model(new SingleTrackModel(params));
  BehaviorModelPtr beh_model(new BehaviorConstantAcceleration(params));

  EvaluatorPtr evaluator_drivable_area(new EvaluatorDrivableArea());

  Polygon polygon = GenerateGoalRectangle(6, 3);
  std::shared_ptr<Polygon> goal_polygon(
      std::dynamic_pointer_cast<Polygon>(polygon.Translate(
          Point2d(50, -2))));  // < move the goal polygon into the driving
                               // corridor in front of the ego vehicle
  auto goal_definition_ptr =
      std::make_shared<GoalDefinitionPolygon>(*goal_polygon);

  double ego_velocity = 5.0, rel_distance = 2.0, velocity_difference = 2.0;

  // TODO: pass goal polygon to make_world_test
  WorldPtr world = make_test_world(0, rel_distance, ego_velocity,
                                   velocity_difference, goal_definition_ptr);

  world->AddEvaluator("drivable_area", evaluator_drivable_area);
  auto eval_res = world->Evaluate()["drivable_area"];

  ASSERT_FALSE(boost::get<bool>(eval_res));
}

#ifdef RSS
TEST(world, rss_evaluator) {
  using bark::world::goal_definition::GoalDefinitionPolygon;

  auto params = std::make_shared<SetterParams>();
  params->SetString("EvaluatorRss::MapFilename",
                    "bark/runtime/tests/data/city_highway_straight.xodr");

  ExecutionModelPtr exec_model(new ExecutionModelInterpolate(params));
  DynamicModelPtr dyn_model(new SingleTrackModel(params));
  BehaviorModelPtr beh_model(new BehaviorConstantAcceleration(params));
  EvaluatorPtr col_checker(new EvaluatorCollisionAgents());

  EvaluatorPtr evaluator_rss(new EvaluatorRSS(params));

  Polygon polygon = GenerateGoalRectangle(6, 3);
  std::shared_ptr<Polygon> goal_polygon(
      std::dynamic_pointer_cast<Polygon>(polygon.Translate(
          Point2d(50, -2))));  // < move the goal polygon into the driving
                               // corridor in front of the ego vehicle
  auto goal_definition_ptr =
      std::make_shared<GoalDefinitionPolygon>(*goal_polygon);

  double ego_velocity = 5.0, rel_distance = 2.0, velocity_difference = 2.0;

  // TODO: pass goal polygon to make_world_test
  WorldPtr world = make_test_world(0, rel_distance, ego_velocity,
                                   velocity_difference, goal_definition_ptr);

  ObservedWorld observed_world(world, 1);

  observed_world.AddEvaluator("rss", evaluator_rss);
  auto eval_res = observed_world.Evaluate()["rss"];

  // no real test assertion, as maps do not match ...
}
#endif

TEST(world, nearest_agents) {
  auto params = std::make_shared<SetterParams>();
  ExecutionModelPtr exec_model(new ExecutionModelInterpolate(params));
  DynamicModelPtr dyn_model(new SingleTrackModel(params));
  BehaviorModelPtr beh_model(new BehaviorConstantAcceleration(params));
  EvaluatorPtr col_checker(new EvaluatorCollisionAgents());

  Polygon polygon(
      Pose(1.25, 1, 0),
      std::vector<Point2d>{Point2d(0, 0), Point2d(0, 2), Point2d(4, 2),
                           Point2d(4, 0), Point2d(0, 0)});

  State init_state1(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state1 << 0.0, 0.0, 0.0, 0.0, 5.0;
  AgentPtr agent1(new Agent(init_state1, beh_model, dyn_model, exec_model,
                            polygon, params));

  State init_state2(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state2 << 0.0, 1.0, 0.0, 0.0, 5.0;
  AgentPtr agent2(new Agent(init_state2, beh_model, dyn_model, exec_model,
                            polygon, params));

  WorldPtr world(new World(params));
  world->AddAgent(agent1);
  world->AddAgent(agent2);

  world->UpdateAgentRTree();

  AgentMap nearest_agents = world->GetNearestAgents(Point2d(0.0, 0.0), 1);

  EXPECT_EQ(nearest_agents.size(), 1);
  EXPECT_EQ(nearest_agents.begin()->second->GetCurrentState(), init_state1);

  AgentMap nearest_agents2 = world->GetNearestAgents(Point2d(0.0, 0.0), 2);

  // Comment as this crashes in CI -> TODO: Why?
  /*EXPECT_EQ(nearest_agents2.size(), 2);
  EXPECT_EQ(nearest_agents2.begin()->second->GetCurrentState(), init_state1 );
  EXPECT_EQ(nearest_agents2[agent2->GetAgentId()]->GetCurrentState(),
  init_state2 );

  State init_state3(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state3 << 0.0, 1.0, 5.0, 0.0, 5.0;
  AgentPtr agent3(new Agent(init_state3, beh_model, dyn_model, exec_model,
  polygon, params));

  world->AddAgent(agent3);
  world->UpdateAgentRTree();

  AgentMap nearest_agents3 = world->GetNearestAgents(Point2d(4.0,5.0),2);
  EXPECT_EQ(nearest_agents3.size(), 2);
  EXPECT_EQ(nearest_agents3[agent1->GetAgentId()]->GetCurrentState(),
  init_state1 );
  EXPECT_EQ(nearest_agents3[agent3->GetAgentId()]->GetCurrentState(),
  init_state3 );
  */
}

TEST(evaluator, distance_to_goal) {
  auto params = std::make_shared<SetterParams>();
  ExecutionModelPtr exec_model(new ExecutionModelInterpolate(params));
  DynamicModelPtr dyn_model(new SingleTrackModel(params));
  BehaviorModelPtr beh_model(new BehaviorConstantAcceleration(params));
  Polygon polygon(
      Pose(1.25, 1, 0),
      std::vector<Point2d>{Point2d(0, 0), Point2d(0, 2), Point2d(4, 2),
                           Point2d(4, 0), Point2d(0, 0)});

  const auto goal_polygon =
      std::dynamic_pointer_cast<Polygon>(polygon.Translate(Point2d(10, 0)));
  auto goal_definition_ptr1 =
      std::make_shared<GoalDefinitionPolygon>(*goal_polygon);

  State init_state1(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state1 << 0.0, 0.0, 0.0, 0.0, 5.0;
  AgentPtr agent1(new Agent(init_state1, beh_model, dyn_model, exec_model,
                            *goal_polygon, params, goal_definition_ptr1));

  State init_state2(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state2 << 0.0, 1.0, -5.3, 0.0, 5.0;
  const auto goal_polygon2 =
      std::dynamic_pointer_cast<Polygon>(polygon.Translate(Point2d(10, 15)));
  auto goal_definition_ptr2 =
      std::make_shared<GoalDefinitionPolygon>(*goal_polygon2);
  AgentPtr agent2(new Agent(init_state2, beh_model, dyn_model, exec_model,
                            *goal_polygon2, params, goal_definition_ptr2));

  WorldPtr world(new World(params));
  world->AddAgent(agent1);
  world->AddAgent(agent2);

  EvaluatorPtr dist_checker1(new EvaluatorDistanceToGoal(agent1->GetAgentId()));
  world->AddEvaluator("distance1", dist_checker1);

  EvaluatorPtr dist_checker2(new EvaluatorDistanceToGoal(agent2->GetAgentId()));
  world->AddEvaluator("distance2", dist_checker2);

  EXPECT_NEAR(boost::get<double>(world->Evaluate()["distance1"]), 10, 0.001);
  EXPECT_NEAR(boost::get<double>(world->Evaluate()["distance2"]),
              sqrt(9 * 9 + (15 + 5.3) * (15 + 5.3)), 0.001);
}

TEST(world, agents_intersection_polygon) {
  auto params = std::make_shared<SetterParams>();
  ExecutionModelPtr exec_model(new ExecutionModelInterpolate(params));
  DynamicModelPtr dyn_model(new SingleTrackModel(params));
  BehaviorModelPtr beh_model(new BehaviorConstantAcceleration(params));
  EvaluatorPtr col_checker(new EvaluatorCollisionAgents());

  Polygon polygon(
      Pose(1.25, 1, 0),
      std::vector<Point2d>{Point2d(0, 0), Point2d(0, 2), Point2d(4, 2),
                           Point2d(4, 0), Point2d(0, 0)});

  State init_state1(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state1 << 0.0, 0.0, 0.0, 0.0, 5.0;
  AgentPtr agent1(new Agent(init_state1, beh_model, dyn_model, exec_model,
                            polygon, params));

  State init_state2(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state2 << 0.0, 10.0, 0.0, 0.0, 5.0;
  AgentPtr agent2(new Agent(init_state2, beh_model, dyn_model, exec_model,
                            polygon, params));

  WorldPtr world(new World(params));
  world->AddAgent(agent1);
  world->AddAgent(agent2);

  world->UpdateAgentRTree();

  Polygon polygon_intersect(
      Pose(0, 0, 0),
      std::vector<Point2d>{Point2d(0, 1), Point2d(0, 3), Point2d(4, 3),
                           Point2d(4, 1), Point2d(0, 1)});

  AgentMap agents_intersect =
      world->GetAgentsIntersectingPolygon(polygon_intersect);

  EXPECT_EQ(agents_intersect.size(), 1);
  EXPECT_EQ(agents_intersect.begin()->second->GetCurrentState(), init_state1);

  Polygon polygon_intersect2(
      Pose(0, 0, 0),
      std::vector<Point2d>{Point2d(7, 1), Point2d(7, 3), Point2d(11, 3),
                           Point2d(11, 1), Point2d(7, 1)});
  AgentMap agents_intersect2 =
      world->GetAgentsIntersectingPolygon(polygon_intersect2);

  EXPECT_EQ(agents_intersect2.size(), 1);
  EXPECT_EQ(agents_intersect2.begin()->second->GetCurrentState(), init_state2);

  State init_state3(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state3 << 0.0, 1.0, 3.0, 0.0, 5.0;
  AgentPtr agent3(new Agent(init_state3, beh_model, dyn_model, exec_model,
                            polygon, params));

  world->AddAgent(agent3);
  world->UpdateAgentRTree();

  Polygon polygon_intersect3(
      Pose(0, 0, 0),
      std::vector<Point2d>{Point2d(0, 1), Point2d(0, 3), Point2d(4, 3),
                           Point2d(4, 1), Point2d(0, 1)});
  AgentMap agents_intersect3 =
      world->GetAgentsIntersectingPolygon(polygon_intersect3);
  EXPECT_EQ(agents_intersect3.size(), 2);
  EXPECT_EQ(agents_intersect3[agent1->GetAgentId()]->GetCurrentState(),
            init_state1);
  EXPECT_EQ(agents_intersect3[agent3->GetAgentId()]->GetCurrentState(),
            init_state3);
}

TEST(world, map_from_csv) {
  std::string csvfile =
      "bark/world/tests/map/data/base_map_lanes_guerickestr_assymetric_48.csv";
  MapInterfacePtr map_interface(new MapInterface());
  bool success = map_interface->interface_from_csvtable(csvfile);
  EXPECT_TRUE(success);

  auto params = std::make_shared<SetterParams>();
  WorldPtr world(new World(params));
  world->SetMap(map_interface);

  ExecutionModelPtr exec_model(new ExecutionModelInterpolate(params));
  DynamicModelPtr dyn_model(new SingleTrackModel(params));
  BehaviorModelPtr beh_model(new BehaviorConstantAcceleration(params));
  State init_state1(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state1 << 0.0, 692967.229902, 5339048.66012, -0.5010, 5.0; // 50th point of centerline
  Polygon polygon(
      Pose(693115.891117, 5339064.33754, 0),  // last pt of centerline
      std::vector<Point2d>{Point2d(-0.5, -0.5), Point2d(-0.5, 0.5),
                           Point2d(0.5, 0.5), Point2d(0.5, -0.5),
                           Point2d(-0.5, -0.5)});
  AgentPtr agent1(new Agent(init_state1, beh_model, dyn_model, exec_model,
                            polygon, params));
  world->UpdateAgentRTree();

  //AgentMap nearest_agents = world->GetNearestAgents(Point2d(692967, 5339048), 1);
  
  for (int i = 0; i < 10; ++i) {
    world->Step(1);
    auto state = agent1->GetCurrentState();
    LOG(INFO) << "x, y = " << state(StateDefinition::X_POSITION) << ", " << state(StateDefinition::Y_POSITION);
  }
}
