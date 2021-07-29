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

class DummySimplexEvaluator : public EvaluatorCollisionEgoAgent {
  // return false if unsafe
 public:
  DummySimplexEvaluator(const AgentId& agent_id)
      : EvaluatorCollisionEgoAgent(agent_id) {}
  virtual EvaluationReturn Evaluate(const World& world) {
    return std::optional<bool>{
      boost::get<bool>(EvaluatorCollisionEgoAgent::Evaluate(world))};
  }
  virtual EvaluationReturn Evaluate(const ObservedWorld& observed_world) {
    return std::optional<bool>{
      boost::get<bool>(EvaluatorCollisionEgoAgent::Evaluate(observed_world))};
  }
};

TEST(behavior_simplex_sampling, violation_threshold) {
  auto params = std::make_shared<SetterParams>();
  double SAFETY_THRESHOLD = 0.1;

  float ego_velocity = 0.0, rel_distance = 0.5, velocity_difference = 0.0;
  float time_step = 0.2;

  WorldPtr world =
      make_test_world(1, rel_distance, ego_velocity, velocity_difference);

  //2) Set observation uncertainty in x-direction only (test world aligned along x-axis)
  params->SetListFloat("ObserverModelParametric::EgoStateDeviationDist::Mean", {0, 0});
  params->SetListListFloat("ObserverModelParametric::EgoStateDeviationDist::Covariance",
                             {{0.00001, 0.0},
                              {0.0, 0.000001}}); // no covariance for ego agent

  params->SetListFloat("ObserverModelParametric::OtherStateDeviationDist::Mean", {0, 0});
  params->SetListListFloat("ObserverModelParametric::OtherStateDeviationDist::Covariance",
                             {{rel_distance, 0.0},
                              {0.0, 0.000001}}); // only covariance for other agents along x-directions

  auto observer_model_parametric= std::make_shared<ObserverModelParametric>(params);
  world->SetObserverModel(observer_model_parametric);

  params->SetReal("BehaviorSimplexSampling::ViolationThreshold", SAFETY_THRESHOLD);
  params->SetInt("BehaviorSimplexSampling::NumSamples", 1000);

  //3) Create behavior and plan
  auto behavior_simplex = BehaviorSimplexSampling(params);

  auto ego_agent = world->GetAgents().begin()->second;
  std::cout << ego_agent->GetCurrentState() << std::endl;
  std::shared_ptr<BehaviorSimplexSampling> behavior_simplex_sampling =
      std::make_shared<BehaviorSimplexSampling>(params);

  //4) Tracking of average triggering of safety maneuver should correspond to chosen safety threshold
  std::shared_ptr<DummySimplexEvaluator> coll_eval =
      std::make_shared<DummySimplexEvaluator>(world->GetAgents().begin()->first);
  behavior_simplex_sampling->SetEvaluator(coll_eval);

  uint max_samples = 100;
  int num_safety_maneuvers = 0;
  for(int i = 0; i < max_samples; ++i) {
      auto observed_world = observer_model_parametric->Observe(world, ego_agent->GetAgentId());
      VLOG(3) << "Other state:" << std::next(observed_world.GetAgents().begin())->second->GetCurrentState() <<
                 "Ego state" << observed_world.GetEgoAgent()->GetCurrentState();
      auto plan_result = behavior_simplex_sampling->Plan(time_step, observed_world);
    if(behavior_simplex_sampling->GetBehaviorRssStatus() == BehaviorRSSConformantStatus::SAFETY_BEHAVIOR) {
        num_safety_maneuvers++;
    }
  }
#ifdef RSS
  EXPECT_NEAR(double(num_safety_maneuvers)/double(max_samples), SAFETY_THRESHOLD, 0.25);
#endif
}

int main(int argc, char** argv) {
  FLAGS_v = 3;
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}