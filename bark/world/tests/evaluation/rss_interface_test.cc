// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <math.h>
#include "gtest/gtest.h"

#ifdef RSS

#include "bark/commons/params/setter_params.hpp"
#include "bark/geometry/polygon.hpp"
#include "bark/models/behavior/constant_acceleration/constant_acceleration.hpp"
#include "bark/models/dynamic/single_track.hpp"
#include "bark/models/execution/interpolation/interpolate.hpp"
#include "bark/world/evaluation/rss/rss_interface.hpp"
#include "bark/world/goal_definition/goal_definition_polygon.hpp"
#include "bark/world/map/map_interface.hpp"
#include "bark/world/map/roadgraph.hpp"
#include "bark/world/objects/agent.hpp"
#include "bark/world/opendrive/opendrive.hpp"
#include "bark/world/tests/make_test_world.hpp"
#include "bark/world/world.hpp"

#include <ad/map/lane/LaneId.hpp>
#include <ad/map/lane/Operation.hpp>
#include <ad/map/match/AdMapMatching.hpp>
#include <ad/map/match/Object.hpp>
#include <ad/map/point/ENUOperation.hpp>
#include <ad/map/route/FullRoute.hpp>
#include <ad/map/route/Planning.hpp>
#include <ad/rss/core/RssCheck.hpp>
#include <ad/rss/map/RssSceneCreation.hpp>
#include <ad/rss/situation/Physics.hpp>
#include <ad/rss/situation/SituationSnapshot.hpp>
#include <ad/rss/situation/SituationType.hpp>
#include <ad/rss/state/ProperResponse.hpp>
#include <ad/rss/state/RssStateOperation.hpp>
#include <ad/rss/state/RssStateSnapshot.hpp>
#include <ad/rss/state/RssStateValidInputRange.hpp>
#include <ad/rss/world/RssDynamics.hpp>

using bark::world::evaluation::RssInterface;
using namespace bark::geometry;
using namespace bark::commons;
using namespace bark::models::behavior;
using namespace bark::models::dynamic;
using namespace bark::models::execution;
using namespace bark::world;
using namespace bark::world::opendrive;
using bark::world::goal_definition::GoalDefinitionPolygon;
using bark::world::map::MapInterface;
using bark::world::map::MapInterfacePtr;
using bark::world::map::Roadgraph;
using bark::world::map::RoadgraphPtr;
using bark::world::opendrive::XodrLaneId;

TEST(rss_interface, test_maps_compatibility) {
  // not all map can be loaded into rss
  RssInterface rss;
  ASSERT_TRUE(rss.InitializeOpenDriveMap(
      "bark/runtime/tests/data/city_highway_straight.xodr"));
  ASSERT_TRUE(rss.InitializeOpenDriveMap(
      "bark/runtime/tests/data/DR_DEU_Merging_MT_v01_centered.xodr"));
  ASSERT_TRUE(rss.InitializeOpenDriveMap(
      "bark/runtime/tests/data/DR_CHN_Merging_ZS_partial_v02.xodr"));
}

TEST(rss_interface, test_rss_check) {
  // set up highway center for testing
  // ===================================================
  auto params = std::make_shared<SetterParams>(false);

  std::shared_ptr<PlanView> p(new PlanView());

  //! add line
  p->AddLine(Point2d(0, -125), M_PI_2, 250);

  //! lane sections
  XodrLaneSectionPtr ls(new XodrLaneSection(0.0f));

  XodrLaneOffset off = {3.6, 0, 0, 0};
  XodrLaneWidth lane_width_1 = {0, 250, off};
  XodrLaneWidth lane_width_2 = {0, 250, off};

  //! Plan View
  XodrLanePtr l_0(new XodrLane(0));
  l_0->SetLine(p->GetReferenceLine());

  XodrLanePtr l_1(new XodrLane(-1));
  XodrLanePtr l_2(new XodrLane(-2));

  l_1 = CreateLaneFromLaneWidth(-1, p->GetReferenceLine(), lane_width_1);
  l_1->SetLaneType(XodrLaneType::DRIVING);
  l_1->SetDrivingDirection(XodrDrivingDirection::FORWARD);

  l_2 = CreateLaneFromLaneWidth(-2, l_1->GetLine(), lane_width_2);
  l_2->SetLaneType(XodrLaneType::DRIVING);
  l_2->SetDrivingDirection(XodrDrivingDirection::FORWARD);

  ls->AddLane(l_0);
  ls->AddLane(l_1);
  ls->AddLane(l_2);

  XodrRoadPtr r(new XodrRoad("highway", 100));
  r->SetPlanView(p);
  r->AddLaneSection(ls);

  OpenDriveMapPtr open_drive_map(new OpenDriveMap());
  open_drive_map->AddRoad(r);

  RoadgraphPtr roadgraph(new Roadgraph());
  MapInterfacePtr map_interface(new MapInterface());
  map_interface->interface_from_opendrive(open_drive_map);

  WorldPtr world(new World(params));
  world->SetMap(map_interface);

  Polygon shape = standard_shapes::CarRectangle();
  Polygon polygon(
      Pose(0, 0., 0),
      std::vector<Point2d>{Point2d(0, 0), Point2d(-1, -1), Point2d(-1, 1),
                           Point2d(1, 1), Point2d(1, -1)});

  std::shared_ptr<Polygon> goal_polygon_1(
      std::dynamic_pointer_cast<Polygon>(polygon.Translate(Point2d(5.5, 120))));
  auto goal_definition_ptr_1 =
      std::make_shared<GoalDefinitionPolygon>(*goal_polygon_1);

  std::shared_ptr<Polygon> goal_polygon_2(
      std::dynamic_pointer_cast<Polygon>(polygon.Translate(Point2d(1.8, 120))));
  auto goal_definition_ptr_2 =
      std::make_shared<GoalDefinitionPolygon>(*goal_polygon_2);

  BehaviorModelPtr beh_model_1(new BehaviorConstantAcceleration(params));
  DynamicModelPtr dyn_model_1(new SingleTrackModel(params));
  ExecutionModelPtr exec_model_1(new ExecutionModelInterpolate(params));
  State state_1(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  state_1 << 0, 5.5, 0, 0, 10;
  objects::AgentPtr agent_1(new Agent(state_1, beh_model_1, dyn_model_1,
                                      exec_model_1, shape, params,
                                      goal_definition_ptr_1, map_interface));
  agent_1->SetAgentId(1);

  BehaviorModelPtr beh_model_2(new BehaviorConstantAcceleration(params));
  DynamicModelPtr dyn_model_2(new SingleTrackModel(params));
  ExecutionModelPtr exec_model_2(new ExecutionModelInterpolate(params));
  State state_2(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  state_2 << 0, 1.8, 0, 0, 10;
  objects::AgentPtr agent_2(new Agent(state_2, beh_model_2, dyn_model_2,
                                      exec_model_2, shape, params,
                                      goal_definition_ptr_2, map_interface));
  agent_1->SetAgentId(2);

  world->AddAgent(agent_1);
  world->UpdateAgentRTree();
  world->AddAgent(agent_2);
  world->UpdateAgentRTree();
  // ===================================================

  world->Step(0.2f);

  RssInterface rss(
      "bark/runtime/tests/data/city_highway_straight.xodr",
      std::vector<float>{3.5, -8., -4., -3., 0.2, -0.8, 0.1, 1.},
      std::unordered_map<AgentId, std::vector<float>>(), 1, 50);

  ::ad::rss::world::WorldModel rss_world;
  WorldPtr cloned_world(world->Clone());
  ObservedWorld observed_world(cloned_world, agent_1->GetAgentId());
  ASSERT_TRUE(rss.GenerateRSSWorld(observed_world, rss_world));

  ::ad::rss::state::RssStateSnapshot snapshot;
  ASSERT_TRUE(rss.RssCheck(rss_world, snapshot));
  ASSERT_EQ(snapshot.individualResponses.size(), 1);
  ASSERT_TRUE(withinValidInputRange(snapshot.individualResponses[0]));
  ASSERT_FALSE(
      ::ad::rss::state::isLongitudinalSafe(snapshot.individualResponses[0]));
  ASSERT_TRUE(::ad::rss::state::isLateralSafe(snapshot.individualResponses[0]));
}

TEST(rss_interface, test_rss_planning_route) {
  auto params = std::make_shared<SetterParams>(false);

  BehaviorModelPtr beh_model(new BehaviorConstantAcceleration(params));
  DynamicModelPtr dyn_model(new SingleTrackModel(params));
  ExecutionModelPtr exec_model(new ExecutionModelInterpolate(params));

  Polygon shape = standard_shapes::CarRectangle();

  Polygon polygon(
      Pose(0, 0., 0),
      std::vector<Point2d>{Point2d(0, 0), Point2d(-1, -1), Point2d(-1, 1),
                           Point2d(1, 1), Point2d(1, -1)});
  std::shared_ptr<Polygon> goal_polygon(
      std::dynamic_pointer_cast<Polygon>(polygon.Translate(Point2d(27, -157))));

  auto goal_definition_ptr =
      std::make_shared<GoalDefinitionPolygon>(*goal_polygon);

  State state(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  state << 0, -46, -168, 0, 10;
  objects::AgentPtr agent(new Agent(state, beh_model, dyn_model, exec_model,
                                    shape, params, goal_definition_ptr));

  WorldPtr world(new World(params));
  world->AddAgent(agent);

  RssInterface rss("bark/runtime/tests/data/DR_CHN_Merging_ZS_partial_v02.xodr",
                   std::vector<float>{3.5, -8., -4., -3., 0.2, -0.8, 0.1, 1.},
                   std::unordered_map<AgentId, std::vector<float>>(), 1, 50);

  Point2d agent_goal;
  boost::geometry::centroid(agent->GetGoalDefinition()->GetShape().obj_, agent_goal);
  State agent_state;
  agent_state = agent->GetCurrentState();
  Polygon agent_shape = agent->GetShape();
  Point2d agent_center =
      Point2d(agent_state(X_POSITION), agent_state(Y_POSITION));

  ::ad::map::match::Object agent_match_object =
      rss.GenerateMatchObject(agent_state, agent_shape);
  ::ad::map::route::FullRoute agent_rss_route;
  ASSERT_TRUE(rss.GenerateRoute(agent_center, agent_goal, agent_match_object,
                                agent_rss_route));
  ASSERT_EQ(agent_rss_route.roadSegments.size(), 2);

  std::vector<::ad::map::lane::LaneId> lanes_id = {147, 10148};
  std::vector<size_t> lanes = {3, 2};

  for (size_t i = 0; i <= 1; ++i) {
    ASSERT_EQ(agent_rss_route.roadSegments[i].drivableLaneSegments.size(),
              lanes[i]);
    ASSERT_EQ(agent_rss_route.roadSegments[i]
                  .drivableLaneSegments[0]
                  .laneInterval.laneId,
              lanes_id[i]);
  }
}

#endif