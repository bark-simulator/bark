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
#include "bark/models/dynamic/single_track.hpp"
#include "bark/models/behavior/dynamic_model/dynamic_model.hpp"
#include "bark/models/dynamic/single_track.hpp"
#include "bark/models/dynamic/single_track_steering_rate.hpp"
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
  DummyObservedWorld(const State& init_state, const ParamsPtr& params, DynamicModelPtr dyn)
      : ObservedWorld(std::make_shared<World>(params), AgentId(0)),
        init_state_(init_state) {

    auto agent = std::make_shared<Agent>(init_state, nullptr, dyn, nullptr,
                                         Polygon(), params);
    agent->SetAgentId(0);
    AddAgent(agent);
  }

  virtual State CurrentEgoState() const { return init_state_; }

  virtual double GetWorldTime() const { return 0.0; }

 private:
  State init_state_;
};


TEST(behavior_motion_primitives_add, behavior_test) {
  auto params = std::make_shared<SetterParams>();
  BehaviorDynamicModel behavior(params);
  Input u(2);
  u << 0, 0;
  behavior.ActionToBehavior(u);

  // single track model
  DynamicModelPtr dyn(new SingleTrackModel(params));
  State init_state0(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state0 << 0.0, 0.0, 0.0, 0.0, 1.0;
  DummyObservedWorld world0(init_state0, params, dyn);
  behavior.ActionToBehavior(u);
  Trajectory traj0 = behavior.Plan(0.5, world0);
  EXPECT_NEAR(traj0(traj0.rows() - 1, StateDefinition::X_POSITION), 0.5, 0.1);
  EXPECT_NEAR(traj0(traj0.rows() - 1, StateDefinition::Y_POSITION), 0.0, 0.1);


  // test single track steering rate model
  DynamicModelPtr dyn_steering_rate(new SingleTrackSteeringRateModel(params));
  State init_state1(static_cast<int>(StateDefinition::MIN_STATE_SIZE) + 1);
  init_state1 << 0.0, 0.0, 0.0, 0.0, 1.0, 0.0;
  DummyObservedWorld world1(init_state1, params, dyn_steering_rate);
  u << 0.5, 3.; // acceleration and steering-rate
  behavior.ActionToBehavior(u);
  Trajectory traj1 = behavior.Plan(0.5, world1);
  EXPECT_NEAR(traj1(traj1.rows() - 1, StateDefinition::X_POSITION), 0.55, 0.1);
  EXPECT_NEAR(traj1(traj1.rows() - 1, StateDefinition::X_POSITION), 0.028, 0.1);


}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
