// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

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


TEST(hypothesis_idm_headway, behavior_hypothesis) {
  // Behavior params
    auto params = std::make_shared<SetterParams>(true);
    // IDM Classic
    params->SetReal("BehaviorIDMClassic::MinimumSpacing", 2.0f);
    params->SetReal("BehaviorIDMClassic::DesiredTimeHeadway", 1.5f);
    params->SetReal("BehaviorIDMClassic::MaxAcceleration",  1.7f);
    params->SetReal("BehaviorIDMClassic::AccelerationLowerBound",  -5.0f);
    params->SetReal("BehaviorIDMClassic::AccelerationUpperBound",  8.0f);
    params->SetReal("BehaviorIDMClassic::DesiredVelocity", 15.0f);
    params->SetReal("BehaviorIDMClassic::ComfortableBrakingAcceleration",  1.67f);
    params->SetReal("BehaviorIDMClassic::MinVelocity", 0.0f);
    params->SetReal("BehaviorIDMClassic::MaxVelocity", 50.0f);
    params->SetInt("BehaviorIDMClassic::Exponent", 4);
    // IDM Stochastic Headway
    params->SetReal("BehaviorIDMStochasticHeadway::HeadwayDistribution::UniformDistribution1D::LowerBound", 1.0f);
    params->SetReal("BehaviorIDMStochasticHeadway::HeadwayDistribution::UniformDistribution1D::UpperBound", 3.0f);
    params->SetDistribution("BehaviorIDMStochasticHeadway::HeadwayDistribution", "UniformDistribution1D");
    // IDM Hypothesis
    params->SetInt("BehaviorHypothesisIDMStochasticHeadway::NumSamples", 10000);
    params->SetInt("BehaviorHypothesisIDMStochasticHeadway::NumBuckets", 100);
    params->SetReal("BehaviorHypothesisIDMStochasticHeadway::BucketsLowerBound", 0.0f);
    params->SetReal("BehaviorHypothesisIDMStochasticHeadway::BucketsUpperBound", 5.0f);

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

  float ego_velocity = 1.0, rel_distance = 7.0,
        velocity_difference = 0.0;
  auto observed_world = make_test_observed_world(
      0, rel_distance, ego_velocity, velocity_difference, goal_definition_ptr);

  // create behavior model
  auto behavior = BehaviorHypothesisIDMStochasticHeadway(params);

  // no other agent in front
  Action action(Continuous1DAction(1.0f));
  auto ego_agent_id = observed_world.GetAgents().begin()->first;
  auto action_prob = behavior.GetProbability(action, observed_world, ego_agent_id);
  EXPECT_NEAR(action_prob, 1, 0.1);

}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}