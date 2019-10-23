// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include <Eigen/Core>
#include "gtest/gtest.h"

#include "modules/geometry/polygon.hpp"
#include "modules/geometry/line.hpp"
#include "modules/geometry/commons.hpp"
#include "modules/models/dynamic/single_track.hpp"
#include "modules/models/behavior/motion_primitives/motion_primitives.hpp"
#include "modules/commons/params/setter_params.hpp"
#include "modules/commons/params/default_params.hpp"
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
  DummyObservedWorld(const State& init_state,
                     Params* params) :
    ObservedWorld(std::make_shared<World>(params), AgentId()),
    init_state_(init_state) { }

  virtual State current_ego_state() const {
    return init_state_;
  }

  virtual double get_world_time() const {
    return 0.0f;
  }
 private:
  State init_state_;
};

TEST(behavior_motion_primitives_add, behavior_test) {
  DefaultParams* params = new DefaultParams();
  DynamicModelPtr dynamics(new SingleTrackModel(params));
  BehaviorMotionPrimitives behavior(dynamics, params);
  Input u(2);
  u << 0, 0;
  behavior.AddMotionPrimitive(u);
}

TEST(behavior_motion_primitives_plan, behavior_test) {
  SetterParams* params = new SetterParams();
  params->set_real("integration_time_delta", 0.01);
  DynamicModelPtr dynamics(new SingleTrackModel(params));

  BehaviorMotionPrimitives behavior(dynamics, params);
  Input u1(2);
  u1 << 2, 0;
  BehaviorMotionPrimitives::MotionIdx idx1 = behavior.AddMotionPrimitive(u1);
  Input u2(2);
  u2 << 0, 1;
  BehaviorMotionPrimitives::MotionIdx idx2 = behavior.AddMotionPrimitive(u2);

  // X Longitudinal with zero velocity
  State init_state(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state << 0.0, 0.0, 0.0, 0.0, 0.0;
  DummyObservedWorld world(init_state, params);

  behavior.ActionToBehavior(idx1);
  Trajectory traj1 = behavior.Plan(0.5, world);
  EXPECT_NEAR(traj1(traj1.rows() - 1,
                    StateDefinition::X_POSITION),
              2/2*0.5*0.5, 0.05);

  // X Longitudinal with nonzero velocity
  init_state << 0.0, 0.0, 0.0, 0.0, 5.0;
  DummyObservedWorld world1(init_state, params);
  behavior.ActionToBehavior(idx1);
  traj1 = behavior.Plan(0.5, world1);
  EXPECT_NEAR(traj1(traj1.rows() - 1,
                    StateDefinition::X_POSITION),
              5.0*0.5 + 2/2*0.5*0.5, 0.1);

  // Y Longitudinal
  init_state << 0.0, 0.0, 0.0, B_PI_2, 0.0;
  DummyObservedWorld world2(init_state, params);
  behavior.ActionToBehavior(idx1);
  traj1 = behavior.Plan(0.5, world2);
  EXPECT_NEAR(traj1(traj1.rows() - 1,
                    StateDefinition::Y_POSITION),
              2/2*0.5*0.5, 0.05);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
