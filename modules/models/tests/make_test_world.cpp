// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <vector>
#include "modules/models/tests/make_test_world.hpp"
#include "modules/geometry/polygon.hpp"
#include "modules/geometry/line.hpp"
#include "modules/geometry/commons.hpp"
#include "modules/models/behavior/idm/idm_classic.hpp"
#include "modules/commons/params/default_params.hpp"
#include "modules/world/observed_world.hpp"
#include "modules/models/execution/interpolation/interpolate.hpp"
#include "modules/models/behavior/constant_velocity/constant_velocity.hpp"

using namespace modules::models::dynamic;
using namespace modules::models::execution;
using namespace modules::commons;
using namespace modules::models::behavior;
using namespace modules::world::map;
using namespace modules::models::dynamic;
using namespace modules::world;
using namespace modules::geometry;
using namespace modules::world::goal_definition;
using modules::world::goal_definition::GoalDefinitionPtr;
using modules::world::map::MapInterface;

WorldPtr modules::models::tests::make_test_world(
  int num_other_agents, double rel_distance,
  double ego_velocity, double velocity_difference,
  const GoalDefinitionPtr& ego_goal_definition) {

  DefaultParams params;
  ExecutionModelPtr exec_model(new ExecutionModelInterpolate(&params));
  DynamicModelPtr dyn_model(nullptr);
  BehaviorModelPtr beh_model_idm(new BehaviorIDMClassic(&params));
  BehaviorModelPtr beh_model_const(new BehaviorConstantVelocity(&params));

  Polygon polygon(Pose(1.25, 1, 0),
    std::vector<Point2d>{Point2d(0, 0),
                         Point2d(0, 2),
                         Point2d(4, 2),
                         Point2d(4, 0),
                         Point2d(0, 0)});

  State init_state1(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state1 << 0.0, 1.0, 0.0, 0.0, ego_velocity;
  AgentPtr agent1(new Agent(init_state1,
                            beh_model_idm,
                            dyn_model,
                            exec_model,
                            polygon,
                            &params,
                            ego_goal_definition));

  State init_state2(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  float rel_dist_vlength = rel_distance + polygon.front_dist_ + polygon.rear_dist_;  // NOLINT
  init_state2 << 0.0, 1.0+rel_dist_vlength, 0.0, 0.0, ego_velocity - velocity_difference;  // NOLINT
  AgentPtr agent2(new Agent(init_state2,
                            beh_model_const,
                            dyn_model,
                            exec_model,
                            polygon,
                            &params));

  State init_state3(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state3 << 0.0, 10.0+rel_dist_vlength, 0.0, 0.0, ego_velocity - velocity_difference;   // NOLINT
  AgentPtr agent3(new Agent(init_state3, beh_model_const, dyn_model, exec_model, polygon, &params));  // NOLINT

  WorldPtr world(new World(&params));
  world->AddAgent(agent1);
  if (num_other_agents == 1) {
    world->AddAgent(agent2);
  } else if (num_other_agents == 2) {
    world->AddAgent(agent3);
  }
  world->UpdateAgentRTree();
  world->SetMap(MapInterfacePtr(new DummyMapInterface()));

  // Define some driving corridor from x=1 to x=20, define it in such a way
  // that no agent collides with the corridor initially
  Line center;
  center.add_point(Point2d(-10, 0));
  center.add_point(Point2d(1, 0));
  center.add_point(Point2d(2, 0));
  center.add_point(Point2d(2000, 0));

  Line left;
  left.add_point(Point2d(-10, 3));
  left.add_point(Point2d(1, 3));
  left.add_point(Point2d(2, 3));
  left.add_point(Point2d(2000, 3));

  Line right;
  right.add_point(Point2d(-10, -3));
  right.add_point(Point2d(1, -3));
  right.add_point(Point2d(2, -3));
  right.add_point(Point2d(2000, -3));

  RoadCorridorPtr road_corridor(new DummyRoadCorridor(center, left, right));
  RoadCorridorPtr road_corridor2(new DummyRoadCorridor(center, left, right));
  RoadCorridorPtr road_corridor3(new DummyRoadCorridor(center, left, right));
  agent1->SetRoadCorridor(road_corridor);
  agent2->SetRoadCorridor(road_corridor2);
  agent3->SetRoadCorridor(road_corridor3);

  return WorldPtr(world->Clone());
}

ObservedWorld modules::models::tests::make_test_observed_world(
  int num_other_agents,
  double rel_distance,
  double ego_velocity,
  double velocity_difference,
  const GoalDefinitionPtr& ego_goal_definition) {
  // Create observed world for first agent
  WorldPtr current_world_state =
    modules::models::tests::make_test_world(num_other_agents,
    rel_distance,
    ego_velocity,
    velocity_difference,
    ego_goal_definition);
  ObservedWorld observed_world(
    current_world_state,
    current_world_state->GetAgents().begin()->second->GetAgentId());
  return observed_world;
}

MapInterface modules::models::tests::make_two_lane_map_interface() {
  using namespace std;
  using namespace modules::world::opendrive;
  using namespace modules::geometry;

  OpenDriveMapPtr map(new OpenDriveMap());

  //! ROAD 1
  PlanViewPtr p(new PlanView());
  p->AddLine(Point2d(0.0f, 0.0f), 0.0f, 10.0f);

  //! XodrLane-Section 1
  XodrLaneSectionPtr ls(new XodrLaneSection(0.0));

  //! PlanView
  XodrLaneOffset off0 = {0.0f, 0.0f, 0.0f, 0.0f};
  XodrLaneWidth lane_width_0 = {0, 10, off0};
  XodrLanePtr lane0 = create_lane_from_lane_width(0,
                                              p->get_reference_line(),
                                              lane_width_0,
                                              0.05);
  lane0->set_lane_type(XodrLaneType::DRIVING);

  //! XodrLane
  XodrLaneOffset off = {1.0f, 0.0f, 0.0f, 0.0f};
  XodrLaneWidth lane_width_1 = {0, 10, off};
  XodrLanePtr lane1 = create_lane_from_lane_width(-1,
                                              p->get_reference_line(),
                                              lane_width_1,
                                              0.05);
  lane1->set_lane_type(XodrLaneType::DRIVING);
  XodrLanePtr lane2 = create_lane_from_lane_width(1,
                                              p->get_reference_line(),
                                              lane_width_1,
                                              0.05);
  lane2->set_lane_type(XodrLaneType::DRIVING);

  ls->AddLane(lane0);
  ls->AddLane(lane1);
  ls->AddLane(lane2);

  XodrRoadPtr r(new XodrRoad("highway", 100));
  r->SetPlanView(p);
  r->AddLaneSection(ls);

  map->add_road(r);


  modules::world::map::MapInterface map_interface;
  map_interface.interface_from_opendrive(map);

  return map_interface;
}

MapInterface modules::models::tests::make_map_interface_two_connected_roads() {
  using namespace modules::world::opendrive;
  using namespace modules::geometry;

  OpenDriveMapPtr map(new OpenDriveMap());

  //! ROAD 1
  PlanViewPtr p0(new PlanView());
  p0->AddLine(Point2d(0.0f, 0.0f), 0.0f, 10.0f);

  //! XodrLane-Section 1
  XodrLaneSectionPtr ls0(new XodrLaneSection(0.0));

  //! PlanView
  XodrLaneOffset off0 = {0.0f, 0.0f, 0.0f, 0.0f};
  XodrLaneWidth lane_width_0 = {0, 10, off0};
  XodrLanePtr lane00 = create_lane_from_lane_width(0,
                                               p0->get_reference_line(),
                                               lane_width_0,
                                               0.05);
  lane00->set_lane_type(XodrLaneType::DRIVING);

  //! XodrLane
  XodrLaneOffset off = {1.0f, 0.0f, 0.0f, 0.0f};
  XodrLaneWidth lane_width_1 = {0, 10, off};
  XodrLanePtr lane01 = create_lane_from_lane_width(1,
                                               p0->get_reference_line(),
                                               lane_width_1,
                                               0.05);
  lane01->set_lane_type(XodrLaneType::DRIVING);

  XodrLanePtr lane02 = create_lane_from_lane_width(2,
                                               p0->get_reference_line(),
                                               lane_width_1,
                                               0.05);
  lane02->set_lane_type(XodrLaneType::DRIVING);

  ls0->AddLane(lane00);
  ls0->AddLane(lane01);
  ls0->AddLane(lane02);

  XodrRoadPtr r0(new XodrRoad("highway", 100));
  r0->SetPlanView(p0);
  r0->AddLaneSection(ls0);

  map->add_road(r0);

  //! ROAD 2
  PlanViewPtr p1(new PlanView());
  p1->AddLine(Point2d(10.0f, 0.0f), 0.0f, 10.0f);

  //! XodrLane-Section 2
  XodrLaneSectionPtr ls1(new XodrLaneSection(0.0f));

  //! PlanView
  XodrLanePtr lane10 = create_lane_from_lane_width(0,
                                               p1->get_reference_line(),
                                               lane_width_0,
                                               0.05);
  lane10->set_lane_type(XodrLaneType::DRIVING);
  lane10->SetLink({0, 0});
  

  //! XodrLane
  XodrLanePtr lane11 = create_lane_from_lane_width(1,
                                               p1->get_reference_line(),
                                               lane_width_1,
                                               0.05);
  lane11->set_lane_type(XodrLaneType::DRIVING);
  lane11->SetLink({1, 1});

  XodrLanePtr lane12 = create_lane_from_lane_width(2,
                                               p1->get_reference_line(),
                                               lane_width_1,
                                               0.05);
  lane12->set_lane_type(XodrLaneType::DRIVING);
  lane12->SetLink({2, 2});

  ls1->AddLane(lane10);
  ls1->AddLane(lane11);
  ls1->AddLane(lane12);

  XodrRoadPtr r1(new XodrRoad("highway", 101));
  r1->SetPlanView(p1);
  r1->AddLaneSection(ls1);

  map->add_road(r1);

  XodrRoadLinkInfo predecessor(100, "road");
  XodrRoadLinkInfo successor(101, "road");
  r1->SetLink(XodrRoadLink(predecessor, successor));

  modules::world::map::MapInterface map_interface;
  map_interface.interface_from_opendrive(map);

  return map_interface;
}