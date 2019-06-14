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
#include "modules/commons/params/default_params.hpp"
#include "modules/world/observed_world.hpp"

using namespace modules::models::dynamic;
using namespace modules::models::execution;
using namespace modules::commons;
using namespace modules::models::behavior;
using namespace modules::models::dynamic;
using namespace modules::world;
using namespace modules::geometry;

class DummyObservedWorld :public ObservedWorld {
  public:
    DummyObservedWorld(State& init_state) : 
              ObservedWorld(World(nullptr), AgentId()),
              init_state_(init_state)  {}

    virtual State get_ego_state() const {
      return init_state_;
    }

    double get_world_time() const {
      return 0.0f;
    }
  private:
    State init_state_;
};



TEST(behavior_motion_primitives_add, behavior_test) {

  DefaultParams params;
  DynamicModelPtr dynamics(new SingleTrackModel());
  BehaviorMotionPrimitives behavior(dynamics, &params);
  Input u(2);
  u << 0, 0;
  behavior.AddMotionPrimitive(u);
}

TEST(behavior_motion_primitives_plan, behavior_test) {
  DefaultParams params;
  DynamicModelPtr dynamics(new SingleTrackModel());

  BehaviorMotionPrimitives behavior(dynamics, &params);
  Input u1(2);
  u1 << 2, 0;
  BehaviorMotionPrimitives::MotionIdx idx1 = behavior.AddMotionPrimitive(u1);
  Input u2(2);
  u2 << 0, 1;
  BehaviorMotionPrimitives::MotionIdx idx2 = behavior.AddMotionPrimitive(u2);
  
  // X Longitudinal
  State init_state(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state << 0.0, 0.0, 0.0, 0.0, 0.0;
  DummyObservedWorld world(init_state);
  behavior.ActionToBehavior(idx1);
  Trajectory traj1 = behavior.Plan(0.5, world);
  EXPECT_NEAR(traj1(traj1.rows()-1,StateDefinition::X_POSITION), 2/2*0.5*0.5, 0.01);

  // Y Longitudinal
  init_state << 0.0, 0.0, 0.0, B_PI_2, 0.0;
  DummyObservedWorld world2(init_state);
  behavior.ActionToBehavior(idx1);
  traj1 = behavior.Plan(0.5, world2);
  EXPECT_NEAR(traj1(traj1.rows()-1,StateDefinition::Y_POSITION), 2/2*0.5*0.5, 0.01);




}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}