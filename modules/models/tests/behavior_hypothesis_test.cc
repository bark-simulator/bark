// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <chrono>
#include <Eigen/Core>
#include "gtest/gtest.h"

#include "modules/commons/params/setter_params.hpp"
#include "modules/geometry/commons.hpp"
#include "modules/geometry/polygon.hpp"
#include "modules/world/observed_world.hpp"
#include "modules/world/tests/make_test_world.hpp"
#include "modules/models/behavior/hypothesis/idm/hypothesis_idm_stochastic_headway.hpp"

using namespace modules::models::dynamic;
using namespace modules::models::execution;
using namespace modules::commons;
using namespace modules::models::behavior;
using namespace modules::world::map;
using namespace modules::models::dynamic;
using namespace modules::world;
using namespace modules::geometry;
using namespace modules::world::tests;

ParamsPtr make_params_hypothesis(float headway_lower, float headway_upper, float fixed_headway,
                                 float acc_lower_bound=-5.0f, float acc_upper_bound=8.0f,
                                 float buckets_lower_bound = -8.0f, float buckets_upper_bound=9.0f) {
    // Behavior params
    auto params = std::make_shared<SetterParams>(true);
    // IDM Classic
    params->SetReal("BehaviorIDMClassic::MinimumSpacing", 2.0f);
    params->SetReal("BehaviorIDMClassic::DesiredTimeHeadway", fixed_headway);
    params->SetReal("BehaviorIDMClassic::MaxAcceleration", 1.7f);
    params->SetReal("BehaviorIDMClassic::AccelerationLowerBound", acc_lower_bound);
    params->SetReal("BehaviorIDMClassic::AccelerationUpperBound", acc_upper_bound);
    params->SetReal("BehaviorIDMClassic::DesiredVelocity", 15.0f);
    params->SetReal("BehaviorIDMClassic::ComfortableBrakingAcceleration",  1.67f);
    params->SetReal("BehaviorIDMClassic::MinVelocity", 0.0f);
    params->SetReal("BehaviorIDMClassic::MaxVelocity", 50.0f);
    params->SetInt("BehaviorIDMClassic::Exponent", 4);
    // IDM Stochastic Headway
    params->SetInt("BehaviorIDMStochasticHeadway::HeadwayDistribution::RandomSeed", 1234);
    params->SetReal("BehaviorIDMStochasticHeadway::HeadwayDistribution::LowerBound", headway_lower);
    params->SetReal("BehaviorIDMStochasticHeadway::HeadwayDistribution::UpperBound", headway_upper);
    params->SetDistribution("BehaviorIDMStochasticHeadway::HeadwayDistribution", "UniformDistribution1D");
    // IDM Hypothesis
    params->SetInt("BehaviorHypothesisIDMStochasticHeadway::NumSamples", 100000);
    params->SetInt("BehaviorHypothesisIDMStochasticHeadway::NumBuckets", 1000);
    params->SetReal("BehaviorHypothesisIDMStochasticHeadway::BucketsLowerBound", buckets_lower_bound);
    params->SetReal("BehaviorHypothesisIDMStochasticHeadway::BucketsUpperBound", buckets_upper_bound);

    return params;
}


TEST(hypothesis_idm_headway, behavior_hypothesis) {
  // Create an observed world with specific goal definition
  Polygon polygon(
      Pose(1, 1, 0),
      std::vector<Point2d>{Point2d(0, 0), Point2d(0, 2), Point2d(2, 2),
                           Point2d(2, 0), Point2d(0, 0)});
  std::shared_ptr<Polygon> goal_polygon(
      std::dynamic_pointer_cast<Polygon>(polygon.Translate(
          Point2d(50, -2))));  // < move the goal polygon into the driving
                               // corridor in front of the ego vehicle
  auto goal_definition_ptr =
      std::make_shared<GoalDefinitionPolygon>(*goal_polygon);

  
  // No other agent in front -> outside max min acceleration limits (exactly on des. velocity)
  {
  auto behavior = BehaviorHypothesisIDMStochasticHeadway(make_params_hypothesis(1.0, 3.0, 1.5));
  const float desired_velocity = behavior.GetDesiredVelocity();

  float ego_velocity = desired_velocity, rel_distance = 20.0, velocity_difference = 0.0;
  auto observed_world = make_test_observed_world(
      0, rel_distance, ego_velocity, velocity_difference, goal_definition_ptr);

  auto traj = behavior.Plan(0.2, observed_world);
  Action action(behavior.GetLastAction());
  auto ego_agent_id = observed_world.GetAgents().begin()->first;
  auto action_prob = behavior.GetProbability(action, observed_world, ego_agent_id);
  EXPECT_NEAR(action_prob, 1, 0.01);

  action_prob = behavior.GetProbability(Action(Continuous1DAction(1.5)), observed_world, ego_agent_id);
  EXPECT_NEAR(action_prob, 0, 0.01);
  }

  // No other agent in front, ego velocity higher than desired velocity
  {
  auto behavior = BehaviorHypothesisIDMStochasticHeadway(make_params_hypothesis(1.0, 3.0, 1.0));
  const float desired_velocity = behavior.GetDesiredVelocity();

  float ego_velocity = desired_velocity + 10, rel_distance = 7.0, velocity_difference = 0.0;
  auto observed_world = make_test_observed_world(
      0, rel_distance, ego_velocity, velocity_difference, goal_definition_ptr);

  auto traj = behavior.Plan(0.2, observed_world);
  Action action2(behavior.GetLastAction());
  auto ego_agent_id = observed_world.GetAgents().begin()->first;
  auto action_prob = behavior.GetProbability(action2, observed_world, ego_agent_id);
  EXPECT_NEAR(action_prob, 1, 0.01);

  action_prob = behavior.GetProbability(Action(boost::get<Continuous1DAction>(action2)+0.4f), observed_world, ego_agent_id);
  EXPECT_NEAR(action_prob, 0, 0.01);
  }

  // Other agent in front, plan action for single parameter (t=1.5 in uniform range (1.0, 3.0))
  // compare to distribution parameter approach, no acceleration limits
  {
  auto params = make_params_hypothesis(1.0, 3.0, 1.5, -100, 100, -110, 110);
  auto behavior = BehaviorHypothesisIDMStochasticHeadway(params);
  const float desired_velocity = behavior.GetDesiredVelocity();

  float ego_velocity = desired_velocity+2, rel_distance = 30.0, velocity_difference = 0.0;
  auto observed_world = make_test_observed_world(
      1, rel_distance, ego_velocity, velocity_difference, goal_definition_ptr);

  auto idm_det_behavior = BehaviorIDMClassic(params);
  auto traj = idm_det_behavior.Plan(0.2, observed_world);
  Action action(idm_det_behavior.GetLastAction());
  auto ego_agent_id = observed_world.GetAgents().begin()->first;
  auto action_prob = behavior.GetProbability(action, observed_world, ego_agent_id);
  EXPECT_NEAR(action_prob, 1/(3.0-1.0), 0.01);
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}