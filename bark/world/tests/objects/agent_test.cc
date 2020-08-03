// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/world/objects/agent.hpp"
#include "bark/commons/params/setter_params.hpp"
#include "bark/geometry/polygon.hpp"
#include "bark/models/behavior/constant_acceleration/constant_acceleration.hpp"
#include "bark/models/dynamic/single_track.hpp"
#include "bark/models/execution/interpolation/interpolate.hpp"
#include "bark/world/world.hpp"
#include "gtest/gtest.h"

using namespace bark::models::dynamic;
using namespace bark::geometry;
using namespace bark::commons;
using namespace bark::world::opendrive;
using namespace bark::models::behavior;
using namespace bark::models::execution;
using namespace bark::world::objects;
using namespace bark::world;

/*
TEST(agent, standard_agent) {

    auto params = std::make_shared<SetterParams>();
    ExecutionModelPtr exec_model(new ExecutionModelInterpolate(params));
    DynamicModelPtr dyn_model(new SingleTrackModel());
    BehaviorModelPtr beh_model(new BehaviorConstantAcceleration(params));
    Polygon
polygon(Pose(1.25,1,0),std::vector<Point2d>{Point2d(0,0),Point2d(0,2),Point2d(4,2),Point2d(4,0),Point2d(0,0)});

    ASSERT_TRUE(polygon.Valid());

    State init_state((int)StateDefinition::MIN_STATE_SIZE);
    init_state << 0.0, 0.0, 0.0, 0.0, 5.0;

    AgentPtr agent(new Agent(init_state,beh_model,dyn_model,exec_model, polygon,
params)); WorldPtr world(new World(params));


       //! new plan view
    PlanView p_1;
    //! add line
    p_1.AddLine(Point2d(0.0f,0.0f), 0.0, 25.0f);
    p_1.AddLine(Point2d(25.0f,0.0f), 0.0, 25.0f);

    //! lane sections
    XodrLaneSection section_1(0.0f);
    XodrLaneSection section_2(30.0f);

    XodrLaneOffset off = {1.5, 0, 0, 0};
    XodrLaneWidths lane_widths_1 = {{0, 30, off}};
    XodrLaneWidths lane_widths_2 = {{0, 30, off}};
    XodrLaneWidths lane_widths_3 = {{30, 50, off}};
    XodrLaneWidths lane_widths_4 = {{30, 50, off}};

    XodrLane l_0 = p_1.create_lane(1, lane_widths_1);
    XodrLane l_1 = p_1.create_lane(-1, lane_widths_2);
    XodrLane l_2 = p_1.create_lane(1, lane_widths_3);
    XodrLane l_3 = p_1.create_lane(-1, lane_widths_4);

    section_1.AddLane(l_0);
    section_1.AddLane(l_1);

    section_2.AddLane(l_2);
    section_2.AddLane(l_3);

    XodrRoad r("highway", 100);
    r.SetPlanView(p_1);
    r.AddLaneSection(section_1);
    r.AddLaneSection(section_2);

    bark::world::opendrive::MapPtr map(new bark::world::opendrive::Map());
    map->AddRoad(r);
    world->SetMap(map);


    world->AddAgent(agent);

    world->Step(1);


    //agent->Move(0.2, world);

    ASSERT_TRUE(true);

}
*/

TEST(agent, PolygonFromState) {
  auto params = std::make_shared<SetterParams>();
  ExecutionModelPtr exec_model(new ExecutionModelInterpolate(params));
  DynamicModelPtr dyn_model(new SingleTrackModel(params));
  BehaviorModelPtr beh_model(new BehaviorConstantAcceleration(params));

  Polygon shape(
      Pose(1.25, 1, 0),
      std::vector<Point2d>{Point2d(0, 0), Point2d(0, 2), Point2d(4, 2),
                           Point2d(4, 0), Point2d(0, 0)});

  State init_state1(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state1 << 0.0, 0.0, 0.0, 0.0, 0.0;
  AgentPtr agent1(
      new Agent(init_state1, beh_model, dyn_model, exec_model, shape, params));

  Polygon poly_out = agent1->GetPolygonFromState(agent1->GetCurrentState());

  EXPECT_TRUE(
      Equals(shape, poly_out));  // we expect true as init_state1 is (0, 0, 0)
                                 // --> transformed polygon is same

  State init_state2(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state2 << 5.0, 2.0, 3.14, 0.0, 0.0;
  AgentPtr agent2(
      new Agent(init_state2, beh_model, dyn_model, exec_model, shape, params));

  Polygon poly_out2 = agent2->GetPolygonFromState(agent2->GetCurrentState());

  EXPECT_FALSE(
      Equals(shape, poly_out2));  // we expect false as init_state2 is non-zero
}
