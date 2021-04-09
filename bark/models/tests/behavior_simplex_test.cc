// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "boost/variant.hpp"
#include "gtest/gtest.h"

#include "bark/commons/params/setter_params.hpp"
#include "bark/models/behavior/behavior_rss/behavior_rss.hpp"
#include "bark/models/behavior/behavior_simplex/behavior_simplex_sampling.hpp"
#include "bark/models/behavior/behavior_safety/behavior_safety.hpp"
#include "bark/models/behavior/idm/idm_classic.hpp"
#include "bark/models/behavior/idm/idm_lane_tracking.hpp"
#include "bark/models/dynamic/single_track.hpp"
#include "bark/models/execution/interpolation/interpolate.hpp"
#include "bark/world/evaluation/base_evaluator.hpp"
#include "bark/world/evaluation/rss/evaluator_rss.hpp"
#include "bark/world/evaluation/evaluator_collision_ego_agent.hpp"
#include "bark/world/observed_world.hpp"
#include "bark/models/observer/observer_model_parametric.hpp"
#include "bark/world/tests/make_test_world.hpp"
#include "bark/world/tests/make_test_xodr_map.hpp"

using bark::commons::SetterParams;
using bark::geometry::Model3D;
using bark::geometry::Polygon;
using bark::world::Agent;
using bark::world::ObservedWorld;
using bark::models::observer::ObserverModelParametric;
using bark::world::World;
using bark::world::WorldPtr;
using bark::world::evaluation::BaseEvaluator;
using bark::world::evaluation::EvaluationReturn;
using bark::world::evaluation::EvaluatorRSS;
using bark::world::evaluation::EvaluatorCollisionEgoAgent;
using bark::world::goal_definition::GoalDefinitionPolygon;
using bark::world::goal_definition::GoalDefinitionPtr;
using bark::world::objects::AgentPtr;
using bark::world::tests::make_test_world;
using bark::world::tests::MakeXodrMapOneRoadTwoLanes;

using namespace bark::models::dynamic;
using namespace bark::models::execution;
using namespace bark::models::behavior;
using namespace bark::world::map;

TEST(behavior_simplex_sampling, violation_threshold) {
  auto params = std::make_shared<SetterParams>();
  double SAFETY_THRESHOLD = 0.1;

  float ego_velocity = 0.0, rel_distance = 7.0, velocity_difference = 0.0;
  float time_step = 0.2;

  WorldPtr world =
      make_test_world(1, rel_distance, ego_velocity, velocity_difference);

  //2 Set observer parameters for x variance to certain value 
  params->SetListFloat("ObserverModelParametric::EgoStateDeviationDist::Mean", {0.7, 1.8, 0.5, 0.5, 0.1});
  params->SetListListFloat("ObserverModelParametric::EgoStateDeviationDist::Covariance",
                             {{0.2, 0.0, 0.0, 0.0, 0.0}, 
                              {0.0, 0.2, 0.0, 0.0, 0.0}, 
                              {0.0, 0.0, 0.2, 0.0, 0.0},
                              {0.0, 0.0, 0.0, 0.2, 0.0},
                              {0.0, 0.0, 0.0, 0.0, 0.1}});

  params->SetListFloat("ObserverModelParametric::OtherStateDeviationDist::Mean", {1.2, 3.5, 0.5, 0.5, 0.2});
  params->SetListListFloat("ObserverModelParametric::OtherStateDeviationDist::Covariance",
                             {{0.3, 0.0, 0.0, 0.0, 0.0}, 
                              {0.0, 2.1, 0.0, 0.0, 0.0}, 
                              {0.0, 0.0, 0.2, 0.0, 0.0},
                              {0.0, 0.0, 0.0, 0.2, 0.0},
                              {0.0, 0.0, 0.0, 0.0, 0.1}});

  auto observer_model_parametric= std::make_shared<ObserverModelParametric>(params);
  world->SetObserverModel(observer_model_parametric);

  params->SetReal("BehaviorSimplexSampling::ViolationThreshold", SAFETY_THRESHOLD);
  //3 Create behavior and plan
  auto behavior_simplex = BehaviorSimplexSampling(params);

  auto ego_agent = world->GetAgents().begin()->second;
  std::cout << ego_agent->GetCurrentState() << std::endl;
  std::shared_ptr<BehaviorSimplexSampling> behavior_simplex_sampling =
      std::make_shared<BehaviorSimplexSampling>(params);

  //4 Get Current Expected violation

  std::shared_ptr<EvaluatorCollisionEgoAgent> coll_eval =
      std::make_shared<EvaluatorCollisionEgoAgent>(world->GetAgents().begin()->first);
  behavior_simplex_sampling->SetEvaluator(coll_eval);

  uint max_samples = 1000;
  int num_safety_maneuvers = 0;
  for(int i = 0; i < max_samples; ++i) {
      auto observed_world = observer_model_parametric->Observe(world, ego_agent->GetAgentId());
      auto plan_result = behavior_simplex_sampling->Plan(time_step, observed_world);
    if(behavior_simplex_sampling->GetBehaviorRssStatus() == BehaviorRSSConformantStatus::SAFETY_BEHAVIOR) {
        num_safety_maneuvers++;
    }
  }
  
  EXPECT_EQ(num_safety_maneuvers/max_samples, SAFETY_THRESHOLD);
}

int main(int argc, char** argv) {
  // FLAGS_v = 5;
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}