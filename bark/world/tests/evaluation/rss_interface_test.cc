// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "gtest/gtest.h"

#ifdef RSS

#include "bark/commons/params/setter_params.hpp"
#include "bark/geometry/polygon.hpp"
#include "bark/models/behavior/constant_acceleration/constant_acceleration.hpp"
#include "bark/models/dynamic/single_track.hpp"
#include "bark/models/execution/interpolation/interpolate.hpp"
#include "bark/world/evaluation/rss/rss_interface.hpp"
#include "bark/world/goal_definition/goal_definition_polygon.hpp"
#include "bark/world/objects/agent.hpp"
#include "bark/world/world.hpp"

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
#include <ad/rss/state/ProperResponse.hpp>
#include <ad/rss/state/RssStateOperation.hpp>
#include <ad/rss/state/RssStateSnapshot.hpp>
#include <ad/rss/world/RssDynamics.hpp>

using bark::world::evaluation::RssInterface;

using namespace bark::geometry;
using namespace bark::commons;
using namespace bark::models::behavior;
using namespace bark::models::dynamic;
using namespace bark::models::execution;
using namespace bark::world;
using bark::world::goal_definition::GoalDefinitionPolygon;

TEST(rss_interface, test_maps_compatibility) {
  // not all map can be loaded into rss
  RssInterface rss;
  ASSERT_TRUE(rss.InitializeOpenDriveMap(
      "bark/runtime/tests/data/city_highway_straight_centered.xodr"));
  ASSERT_TRUE(rss.InitializeOpenDriveMap(
      "bark/runtime/tests/data/DR_DEU_Merging_MT_v01_centered.xodr"));
  ASSERT_TRUE(rss.InitializeOpenDriveMap(
      "bark/runtime/tests/data/DR_CHN_Merging_ZS_partial_v02.xodr"));
}

TEST(rss_interface, test_rss_check) {
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
      std::dynamic_pointer_cast<Polygon>(polygon.Translate(Point2d(-16, 108))));

  auto goal_definition_ptr =
      std::make_shared<GoalDefinitionPolygon>(*goal_polygon);

  State state(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  state << 0, 68, 108, 0, 10;
  objects::AgentPtr agent(new Agent(state, beh_model, dyn_model, exec_model,
                                    shape, params, goal_definition_ptr));

  WorldPtr world(new World(params));
  world->AddAgent(agent);

  RssInterface rss(
      "bark/runtime/tests/data/DR_DEU_Merging_MT_v01_centered.xodr",
      std::vector<float>{3.5, -8., -4., -3., 0.2, -0.8, 0.1, 1.},
      std::unordered_map<AgentId, std::vector<float>>(), 1, 50);

  ::ad::rss::world::WorldModel rss_world;
  ASSERT_TRUE(rss.GenerateRSSWorld(*world, agent->GetAgentId(), rss_world));

  ::ad::rss::state::RssStateSnapshot snapshot;
  ASSERT_TRUE(rss.RssCheck(rss_world, snapshot));
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
  bg::centroid(agent->GetGoalDefinition()->GetShape().obj_, agent_goal);
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
}

#endif