// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <string>

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
#include "modules/world/goal_definition/goal_definition_polygon.hpp"

#include "modules/world/evaluation/rss_interface.hpp"
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

using modules::world::evaluation::RssInterface;
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
using modules::world::goal_definition::GoalDefinitionPolygon;
// using ::ad::map::point::ENUCoordinate;
// using ::ad::physics::Acceleration;
// using ::ad::physics::Distance;
// using ::ad::physics::Duration;
// using ::ad::physics::Speed;
typedef std::map<AgentId, AgentPtr> AgentMap;
using objects::AgentPtr;

TEST(rss_interface,load_odr) {
  std::string map_path = "modules/runtime/tests/data/centered_city_highway_straight.xodr";
  RssInterface rss;
  ASSERT_TRUE(rss.initializeOpenDriveMap(map_path));
}

TEST(rss_interface,test_lon_safety_check){
  auto params = std::make_shared<DefaultParams>();
  ExecutionModelPtr exec_model(new ExecutionModelInterpolate(params));
  DynamicModelPtr dyn_model(new SingleTrackModel(params));
  BehaviorModelPtr beh_model(new BehaviorConstantVelocity(params));

  //! new plan view
  std::shared_ptr<PlanView> p_1(new PlanView());

  //! add line //TODO
  p_1->AddLine(Point2d(0.0f, 0.0f), 0.0, 25.0f);
  p_1->AddLine(Point2d(25.0f, 0.0f), 0.0, 25.0f);

  //! lane sections
  std::shared_ptr<XodrLaneSection> section_1(new XodrLaneSection(250));

  XodrLaneOffset off = {0, 0, 0, 0};
  XodrLaneWidth lane_width_1 = {0, 3.6, off};
  XodrLaneWidth lane_width_2 = {0, 3.6, off};

  std::shared_ptr<XodrLane> l_0(new XodrLane(1));
  std::shared_ptr<XodrLane> l_1(new XodrLane(-1));

  l_0 = CreateLaneFromLaneWidth(1, p_1->GetReferenceLine(), lane_width_1);
  l_1 = CreateLaneFromLaneWidth(-1, p_1->GetReferenceLine(), lane_width_2);

  section_1->AddLane(l_0);
  section_1->AddLane(l_1);

  std::shared_ptr<XodrRoad> r(new XodrRoad("highway", 250));

  r->SetPlanView(p_1);
  r->AddLaneSection(section_1);

  modules::world::opendrive::OpenDriveMapPtr open_drive_map(new modules::world::opendrive::OpenDriveMap());
  open_drive_map->AddRoad(r);

  RoadgraphPtr roadgraph(new Roadgraph());
  MapInterfacePtr map_interface(new MapInterface());
  map_interface->SetOpenDriveMap(open_drive_map);
  map_interface->SetRoadgraph(roadgraph);

  Polygon shape(Pose(1.25, 1, 0), std::vector<Point2d>{Point2d(0, 0), Point2d(0, 2), Point2d(4, 2), Point2d(4, 0), Point2d(0, 0)});
  Polygon polygon(Pose(0, 0., 0), std::vector<Point2d>{Point2d(0, 0), Point2d(-1, -1), Point2d(-1, 1), Point2d(1, 1), Point2d(1, -1)});
  std::shared_ptr<Polygon> goal_polygon(std::dynamic_pointer_cast<Polygon>(polygon.Translate(Point2d(1.8,120)))); // < move the goal polygon into the driving corridor in front of the ego vehicle
  auto goal_definition_ptr = std::make_shared<GoalDefinitionPolygon>(*goal_polygon);

  State init_state1(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state1 << 0., 1.8, -120., 0.0, 10.0;
  AgentPtr agent1(new Agent(init_state1, beh_model, dyn_model, exec_model, shape, params,goal_definition_ptr));

  State init_state2(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state2 << 0., 1.8, -50., 0., 5.;
  AgentPtr agent2(new Agent(init_state2, beh_model, dyn_model, exec_model, shape, params,goal_definition_ptr));

  WorldPtr world(new World(params));
  world->AddAgent(agent1);
  world->AddAgent(agent2);
  world->SetMap(map_interface);

  std::string map_path = "modules/runtime/tests/data/centered_city_highway_straight.xodr";
  RssInterface rss;
  rss.initializeOpenDriveMap(map_path);

  auto matched_object = rss.GetMatchObject(init_state1, shape, ::ad::physics::Distance(2.0));
  Point2d agent_center =
      Point2d(init_state1(StateDefinition::X_POSITION), init_state1(StateDefinition::Y_POSITION));
  
  // map::RoadCorridorPtr agent_road_corridor = agent1->GetRoadCorridor();
  // map::LaneId agent_lane_id = world->GetMap()->FindCurrentLane(agent_center);
  // map::LaneCorridorPtr agent_lane_corridor =
  //     agent_road_corridor->GetLaneCorridor(agent_lane_id);
  // ::ad::map::route::FullRoute agent_route =
  //     rss.GenerateRoute(agent_center, agent_lane_corridor, matched_object);
}