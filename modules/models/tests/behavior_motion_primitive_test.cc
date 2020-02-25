// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <Eigen/Core>
#include "gtest/gtest.h"

#include "modules/commons/params/default_params.hpp"
#include "modules/commons/params/setter_params.hpp"
#include "modules/geometry/commons.hpp"
#include "modules/geometry/line.hpp"
#include "modules/geometry/polygon.hpp"
#include "modules/models/behavior/motion_primitives/continuous_actions.hpp"
#include "modules/models/behavior/motion_primitives/macro_actions.hpp"
#include "modules/models/behavior/motion_primitives/primitives.hpp"
#include "modules/models/dynamic/single_track.hpp"
#include "modules/world/observed_world.hpp"

using namespace modules::models::dynamic;
using namespace modules::models::execution;
using namespace modules::commons;
using namespace modules::models::behavior;
using namespace modules::models::dynamic;
using namespace modules::world;
using namespace modules::geometry;

class DummyObservedWorld : public ObservedWorld {
 public:
  DummyObservedWorld(const State& init_state, const ParamsPtr& params)
      : ObservedWorld(std::make_shared<World>(params), AgentId()),
        init_state_(init_state) {}

  virtual State CurrentEgoState() const { return init_state_; }

  virtual double GetWorldTime() const { return 0.0f; }

 private:
  State init_state_;
};

TEST(behavior_motion_primitives_add, behavior_test) {
  auto params = std::make_shared<DefaultParams>();
  DynamicModelPtr dynamics(new SingleTrackModel(params));
  BehaviorMPContinuousActions behavior(dynamics, params);
  Input u(2);
  u << 0, 0;
  behavior.AddMotionPrimitive(u);
}

TEST(behavior_motion_primitives_plan, behavior_test) {
  auto params = std::make_shared<SetterParams>();
  params->SetReal("integration_time_delta", 0.01);
  DynamicModelPtr dynamics(new SingleTrackModel(params));

  BehaviorMPContinuousActions behavior(dynamics, params);
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
  using modules::models::behavior::primitives::PrimitiveConstAccStayLane;
  auto params = std::make_shared<DefaultParams>();
  DynamicModelPtr dynamics(new SingleTrackModel(params));
  PrimitiveConstAccStayLane primitive(params, dynamics, 0, 0.1);

  State init_state(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state << 0.0, 0.0, 0.0, 0.0, 5.0;
  auto world1 = std::make_shared<DummyObservedWorld>(init_state, params);
  EXPECT_TRUE(primitive.IsPreConditionSatisfied(world1));
  // auto traj = primitive.Plan(0.5, world1);
}

TEST(primitive_change_left, behavior_test) {
  using modules::models::behavior::primitives::PrimitiveConstAccChangeToLeft;
  auto params = std::make_shared<DefaultParams>();
  DynamicModelPtr dynamics(new SingleTrackModel(params));
  PrimitiveConstAccChangeToLeft primitive(params, dynamics, 0.1);

  State init_state(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state << 0.0, 0.0, 0.0, 0.0, 5.0;
  auto world1 = std::make_shared<DummyObservedWorld>(init_state, params);
  // EXPECT_FALSE(primitive.IsPreConditionSatisfied(world1));
  // auto traj = primitive.Plan(0.5, world1);
}

TEST(primitive_gap_keeping, behavior_test) {
  using modules::models::behavior::primitives::PrimitiveGapKeeping;
  auto params = std::make_shared<DefaultParams>();
  DynamicModelPtr dynamics(new SingleTrackModel(params));
  PrimitiveGapKeeping primitive(params, dynamics);

  State init_state(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state << 0.0, 0.0, 0.0, 0.0, 5.0;
  auto world1 = std::make_shared<DummyObservedWorld>(init_state, params);
  // EXPECT_FALSE(primitive.IsPreConditionSatisfied(world1));
  // auto traj = primitive.Plan(0.5, world1);
}

TEST(macro_actions, behavior_test) {
  using namespace modules::models::behavior::primitives;
  using modules::models::behavior::primitives::PrimitiveConstAccStayLane;

  auto params = std::make_shared<DefaultParams>();
  params->SetReal("integration_time_delta", 0.01);
  DynamicModelPtr dynamics(new SingleTrackModel(params));

  std::vector<std::shared_ptr<Primitive>> prim_vec;
  
  auto primitive =
      std::make_shared<PrimitiveConstAccStayLane>(params, dynamics, 0, 0.1);
  prim_vec.push_back(primitive);

  auto primitive_left =
      std::make_shared<PrimitiveConstAccChangeToLeft>(params, dynamics, 0.1);
  prim_vec.push_back(primitive_left);

  auto primitive_right =
      std::make_shared<PrimitiveConstAccChangeToRight>(params, dynamics, 0.1);
  prim_vec.push_back(primitive_right);

  BehaviorMPMacroActions behavior(dynamics, params);
  for (auto& p : prim_vec) {
    auto idx = behavior.AddMotionPrimitive(primitive);
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
