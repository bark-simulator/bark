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
#include "bark/geometry/commons.hpp"
#include "bark/geometry/line.hpp"
#include "bark/geometry/polygon.hpp"
#include "bark/models/behavior/motion_primitives/continuous_actions.hpp"
#include "bark/models/behavior/motion_primitives/macro_actions.hpp"
#include "bark/models/behavior/motion_primitives/primitives/primitive.hpp"
#include "bark/models/behavior/motion_primitives/primitives/primitive_const_acc_change_to_left.hpp"
#include "bark/models/behavior/motion_primitives/primitives/primitive_const_acc_change_to_right.hpp"
#include "bark/models/behavior/motion_primitives/primitives/primitive_const_acc_stay_lane.hpp"
#include "bark/models/behavior/motion_primitives/primitives/primitive_gap_keeping.hpp"
#include "bark/models/dynamic/single_track.hpp"
#include "bark/world/observed_world.hpp"
#include "bark/world/tests/make_test_world.hpp"

using namespace bark::models::dynamic;
using namespace bark::models::execution;
using namespace bark::commons;
using namespace bark::models::behavior;
using namespace bark::models::dynamic;
using namespace bark::world;
using namespace bark::geometry;
using namespace bark::world::tests;

class DummyObservedWorld : public ObservedWorld {
 public:
  DummyObservedWorld(const State& init_state, const ParamsPtr& params)
      : ObservedWorld(std::make_shared<World>(params), AgentId(0)),
        init_state_(init_state) {
    DynamicModelPtr dynamics(new SingleTrackModel(params));
    auto agent = std::make_shared<Agent>(init_state, nullptr, dynamics, nullptr,
                                         Polygon(), params);
    agent->SetAgentId(0);
    AddAgent(agent);
  }

  virtual State CurrentEgoState() const { return init_state_; }

  virtual double GetWorldTime() const { return 0.0f; }

 private:
  State init_state_;
};

AdjacentLaneCorridors GetCorridors(const ObservedWorld& observed_world) {
  auto target_corridor = observed_world.GetLaneCorridor();
  AdjacentLaneCorridors adjacent_corridors;
  auto ego_pos = observed_world.CurrentEgoPosition();
  auto road_corridor = observed_world.GetRoadCorridor();
  std::tie(adjacent_corridors.left, adjacent_corridors.right) =
      road_corridor->GetLeftRightLaneCorridor(ego_pos);
  adjacent_corridors.current = target_corridor;
  return adjacent_corridors;
}

TEST(behavior_motion_primitives_add, behavior_test) {
  auto params = std::make_shared<SetterParams>();
  BehaviorMPContinuousActions behavior(params);
  Input u(2);
  u << 0, 0;
  behavior.AddMotionPrimitive(u);
}

TEST(behavior_motion_primitives_plan, behavior_test) {
  auto params = std::make_shared<SetterParams>();
  params->SetReal("integration_time_delta", 0.01);
  DynamicModelPtr dynamics(new SingleTrackModel(params));

  BehaviorMPContinuousActions behavior(params);
  Input u1(2);
  u1 << 2, 0;
  BehaviorMPContinuousActions::MotionIdx idx1 = behavior.AddMotionPrimitive(u1);
  Input u2(2);
  u2 << 0, 1;
  BehaviorMPContinuousActions::MotionIdx idx2 = behavior.AddMotionPrimitive(u2);
  Input u3(2);
  u3 << 0, 0;
  BehaviorMPContinuousActions::MotionIdx idx3 = behavior.AddMotionPrimitive(u3);

  // X Longitudinal with zero velocity
  State init_state(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state << 0.0, 0.0, 0.0, 0.0, 0.0;
  DummyObservedWorld world(init_state, params);

  behavior.ActionToBehavior(idx1);
  Trajectory traj1 = behavior.Plan(0.5, world);
  EXPECT_NEAR(traj1(traj1.rows() - 1, StateDefinition::X_POSITION),
              2 / 2 * 0.5 * 0.5, 0.05);

  // X Longitudinal with nonzero velocity
  init_state << 0.0, 0.0, 0.0, 0.0, 5.0;
  DummyObservedWorld world1(init_state, params);
  behavior.ActionToBehavior(idx1);
  traj1 = behavior.Plan(0.5, world1);
  EXPECT_NEAR(traj1(traj1.rows() - 1, StateDefinition::X_POSITION),
              5.0 * 0.5 + 2 / 2 * 0.5 * 0.5, 0.1);

  // Y Longitudinal
  init_state << 0.0, 0.0, 0.0, B_PI_2, 0.0;
  DummyObservedWorld world2(init_state, params);
  behavior.ActionToBehavior(idx1);
  traj1 = behavior.Plan(0.5, world2);
  EXPECT_NEAR(traj1(traj1.rows() - 1, StateDefinition::Y_POSITION),
              2 / 2 * 0.5 * 0.5, 0.05);

  // X Constant motion
  init_state << 0.0, 0.0, 0.0, 0.0, 2.0;
  DummyObservedWorld world3(init_state, params);
  behavior.ActionToBehavior(idx3);
  Trajectory traj3 = behavior.Plan(0.5, world3);
  EXPECT_NEAR(traj3(traj3.rows() - 1, StateDefinition::X_POSITION), 0.5 * 2,
              0.005);
}

TEST(primitive_constant_acceleration, behavior_test) {
  using bark::models::behavior::primitives::AdjacentLaneCorridors;
  using bark::models::behavior::primitives::PrimitiveConstAccStayLane;
  auto params = std::make_shared<SetterParams>();
  DynamicModelPtr dynamics(new SingleTrackModel(params));
  PrimitiveConstAccStayLane primitive(params, 0);

  auto world1 = make_test_observed_world(0, 0.0, 5.0, 0.0);
  std::const_pointer_cast<Agent>(world1.GetEgoAgent())
      ->SetDynamicModel(dynamics);
  AdjacentLaneCorridors corridors = GetCorridors(world1);
  EXPECT_TRUE(primitive.IsPreConditionSatisfied(world1, corridors));
  auto traj1 = primitive.Plan(0.5, world1, corridors.current);
  EXPECT_NEAR(traj1(traj1.rows() - 1, StateDefinition::X_POSITION),
              traj1(0, StateDefinition::X_POSITION) + 5.0 * 0.5, 0.1);

  PrimitiveConstAccStayLane dec_primitive(params, -5.0);
  EXPECT_TRUE(dec_primitive.IsPreConditionSatisfied(world1, corridors));
  auto traj2 = dec_primitive.Plan(0.5, world1, corridors.current);
  EXPECT_NEAR(traj2(traj2.rows() - 1, StateDefinition::X_POSITION),
              traj2(0, StateDefinition::X_POSITION) + 5.0 * 0.5 +
                  -5.0 / 2.0 * 0.5 * 0.5,
              0.1);
  auto world2 = make_test_observed_world(0, 0.0, 0.0, 0.0);
  std::const_pointer_cast<Agent>(world2.GetEgoAgent())
      ->SetDynamicModel(dynamics);
  AdjacentLaneCorridors corridors2 = GetCorridors(world2);
  EXPECT_FALSE(dec_primitive.IsPreConditionSatisfied(world2, corridors2));

  PrimitiveConstAccStayLane acc_primitive(params, 4.0);
  EXPECT_TRUE(acc_primitive.IsPreConditionSatisfied(world1, corridors));
  auto traj3 = acc_primitive.Plan(0.5, world1, corridors.current);
  EXPECT_NEAR(
      traj3(traj3.rows() - 1, StateDefinition::X_POSITION),
      traj3(0, StateDefinition::X_POSITION) + 5.0 * 0.5 + 4.0 / 2.0 * 0.5 * 0.5,
      0.1);
}

TEST(primitive_change_left, behavior_test) {
  using bark::models::behavior::primitives::PrimitiveConstAccChangeToLeft;
  auto params = std::make_shared<SetterParams>();
  PrimitiveConstAccChangeToLeft primitive(params);
  auto world = MakeTestWorldHighway();
  auto observed_worlds = world->Observe({1, 4});
  auto corridors0 = GetCorridors(observed_worlds[0]);
  EXPECT_FALSE(
      primitive.IsPreConditionSatisfied(observed_worlds[0], corridors0));
  auto corridors1 = GetCorridors(observed_worlds[1]);
  EXPECT_TRUE(
      primitive.IsPreConditionSatisfied(observed_worlds[1], corridors1));
  auto target_corridor =
      primitive.SelectTargetCorridor(observed_worlds[1], corridors1);
  EXPECT_EQ(target_corridor, corridors1.left);
}

TEST(primitive_change_right, behavior_test) {
  using bark::models::behavior::primitives::PrimitiveConstAccChangeToRight;
  auto params = std::make_shared<SetterParams>();
  PrimitiveConstAccChangeToRight primitive(params);
  auto world = MakeTestWorldHighway();
  auto observed_worlds = world->Observe({1, 4});
  auto corridors0 = GetCorridors(observed_worlds[0]);
  EXPECT_TRUE(
      primitive.IsPreConditionSatisfied(observed_worlds[0], corridors0));
  auto target_corridor =
      primitive.SelectTargetCorridor(observed_worlds[0], corridors0);
  EXPECT_EQ(target_corridor, corridors0.right);
  auto corridors1 = GetCorridors(observed_worlds[1]);
  EXPECT_FALSE(
      primitive.IsPreConditionSatisfied(observed_worlds[1], corridors1));
}

TEST(primitive_gap_keeping, precondition_test) {
  using bark::models::behavior::primitives::PrimitiveGapKeeping;
  auto params = std::make_shared<SetterParams>();
  DynamicModelPtr dynamics(new SingleTrackModel(params));
  PrimitiveGapKeeping primitive(params);

  State init_state(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state << 0.0, 0.0, 0.0, 0.0, 5.0;
  auto world1 = DummyObservedWorld(init_state, params);
  AdjacentLaneCorridors corridors = {nullptr, nullptr, nullptr};
  EXPECT_TRUE(primitive.IsPreConditionSatisfied(world1, corridors));
  // auto traj = primitive.Plan(0.5, world1);
}

TEST(macro_actions, behavior_test) {
  using namespace bark::models::behavior::primitives;
  using bark::models::behavior::primitives::PrimitiveConstAccStayLane;

  auto params = std::make_shared<SetterParams>();
  params->SetReal("integration_time_delta", 0.01);

  std::vector<std::shared_ptr<Primitive>> prim_vec;

  auto primitive = std::make_shared<PrimitiveConstAccStayLane>(params, 0.0);
  prim_vec.push_back(primitive);

  auto primitive_left = std::make_shared<PrimitiveConstAccChangeToLeft>(params);
  prim_vec.push_back(primitive_left);

  auto primitive_right =
      std::make_shared<PrimitiveConstAccChangeToRight>(params);
  prim_vec.push_back(primitive_right);

  BehaviorMPMacroActions behavior(params);
  for (const auto& p : prim_vec) {
    behavior.AddMotionPrimitive(p);
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
