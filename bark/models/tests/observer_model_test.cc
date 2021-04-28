// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#include "gtest/gtest.h"
#include "bark/models/observer/observer_model.hpp"
#include "bark/models/observer/observer_model_none.hpp"
#include "bark/models/observer/observer_model_parametric.hpp"
#include "bark/world/tests/make_test_world.hpp"
#include "bark/commons/params/setter_params.hpp"

using namespace bark::commons;
using namespace bark::geometry;
using namespace bark::geometry::standard_shapes;
using namespace bark::world::map;
using namespace bark::world::objects;
using namespace bark::world::tests;
using namespace bark::world;
using namespace bark::models::dynamic;
using namespace bark::models::observer;
using namespace bark::models::behavior;
using namespace bark::world::tests;
using namespace bark::models::execution;

TEST(observer_model_none, base_test) {
  using bark::models::observer::ObserverModelNone;
  auto params = std::make_shared<SetterParams>();
  ObserverModelNone observer_none(params);

  Polygon polygon = GenerateGoalRectangle(6,3);
  std::shared_ptr<Polygon> goal_polygon(
    std::dynamic_pointer_cast<Polygon>(polygon.Translate(
      Point2d(50, -2))));  // < move the goal polygon into the driving
                            // corridor in front of the ego vehicle
  auto goal_definition_ptr =
      std::make_shared<GoalDefinitionPolygon>(*goal_polygon);
  double ego_velocity = 5.0, rel_distance = 2.0, velocity_difference = 2.0;
  WorldPtr world = make_test_world(0, rel_distance, ego_velocity,
                                   velocity_difference, goal_definition_ptr);

  ObservedWorld observed_world = observer_none.Observe(world, AgentId(1));

  // NOTE: assert ObservedWorld is generated for the correct AgentId
  EXPECT_EQ(observed_world.GetEgoAgentId(), AgentId(1));
}

TEST(observer_model_parametric, position_deviation_test) {
  auto params = std::make_shared<SetterParams>();

  // Create World with known agent positions
  // Frenet coordinates oriented along cartesian coordinate system
  Polygon polygon = GenerateGoalRectangle(6,3);
  std::shared_ptr<Polygon> goal_polygon(
    std::dynamic_pointer_cast<Polygon>(polygon.Translate(
      Point2d(50, -2))));  // < move the goal polygon into the driving
                            // corridor in front of the ego vehicle
  auto goal_definition_ptr =
      std::make_shared<GoalDefinitionPolygon>(*goal_polygon);
  double ego_velocity = 5.0, rel_distance = 2.0, velocity_difference = 2.0;
  WorldPtr world = make_test_world(2, rel_distance, ego_velocity,
                                   velocity_difference, goal_definition_ptr);

  params->SetListFloat("ObserverModelParametric::EgoStateDeviationDist::Mean", {0.7, 1.8});
  params->SetListListFloat("ObserverModelParametric::EgoStateDeviationDist::Covariance",
                             {{0.2, 0.0}, 
                              {0.0, 0.2}});
  
  params->SetListFloat("ObserverModelParametric::OtherStateDeviationDist::Mean", {1.2, 3.5});
  params->SetListListFloat("ObserverModelParametric::OtherStateDeviationDist::Covariance",
                             {{0.3, 0.0}, 
                              {0.0, 2.1}});

  ObserverModelParametric observer_parametric(params);

  const auto agent1 = world->GetAgents().begin()->second;
  const auto agent2 = std::next(world->GetAgents().begin())->second;
  const auto agent3 = std::next(world->GetAgents().begin(), 2)->second;

  unsigned int num_samples = 10000;
  const auto current_state_agent1 = agent1->GetCurrentState();
  const auto current_state_agent2 = agent2->GetCurrentState();
  const auto current_state_agent3 = agent3->GetCurrentState();

  State mean_deviation1(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  State mean_deviation2(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  State mean_deviation3(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  mean_deviation1.setZero();
  mean_deviation2.setZero();
  mean_deviation3.setZero();

  for (int i = 1; i <= num_samples; ++i) {
    const auto observed_world = observer_parametric.Observe(world, agent2->GetAgentId());
    const auto obs_state1 = observed_world.GetAgent(agent1->GetAgentId())->GetCurrentState();
    const auto obs_state2 = observed_world.GetAgent(agent2->GetAgentId())->GetCurrentState();
    const auto obs_state3 = observed_world.GetAgent(agent3->GetAgentId())->GetCurrentState();

    // Frenet state is aligned along coordinate system, so no transformation required
    mean_deviation1 = (mean_deviation1*double(i-1) + (obs_state1 - current_state_agent1).cwiseAbs())/double(i);
    mean_deviation2 = (mean_deviation2*double(i-1) + (obs_state2 - current_state_agent2).cwiseAbs())/double(i);
    mean_deviation3 = (mean_deviation3*double(i-1) + (obs_state3 - current_state_agent3).cwiseAbs())/double(i);
  }

  // Compare x
  auto index = static_cast<int>(StateDefinition::X_POSITION);
  EXPECT_NEAR(mean_deviation1(index), params->GetListFloat("ObserverModelParametric::OtherStateDeviationDist::Mean", "", {}).at(0), 0.05);
  EXPECT_NEAR(mean_deviation2(index), params->GetListFloat("ObserverModelParametric::EgoStateDeviationDist::Mean", "", {}).at(0), 0.05);
  EXPECT_NEAR(mean_deviation3(index), params->GetListFloat("ObserverModelParametric::OtherStateDeviationDist::Mean", "", {}).at(0), 0.05);

  // Compare y
  index = static_cast<int>(StateDefinition::Y_POSITION);
  EXPECT_NEAR(mean_deviation1(index), params->GetListFloat("ObserverModelParametric::OtherStateDeviationDist::Mean", "", {}).at(1), 0.05);
  EXPECT_NEAR(mean_deviation2(index), params->GetListFloat("ObserverModelParametric::EgoStateDeviationDist::Mean", "", {}).at(1), 0.05);
  EXPECT_NEAR(mean_deviation3(index), params->GetListFloat("ObserverModelParametric::OtherStateDeviationDist::Mean", "", {}).at(1), 0.05);
}

TEST(observer_model_parametric, velocity_deviation_test) {
  auto params = std::make_shared<SetterParams>();

  // Create World with known agent positions
  // Frenet coordinates oriented along cartesian coordinate system
  Polygon polygon = GenerateGoalRectangle(6,3);
  std::shared_ptr<Polygon> goal_polygon(
    std::dynamic_pointer_cast<Polygon>(polygon.Translate(
      Point2d(50, -2))));  // < move the goal polygon into the driving
                            // corridor in front of the ego vehicle
  auto goal_definition_ptr =
      std::make_shared<GoalDefinitionPolygon>(*goal_polygon);
  double ego_velocity = 5.0, rel_distance = 2.0, velocity_difference = 2.0;
  WorldPtr world = make_test_world(2, rel_distance, ego_velocity,
                                   velocity_difference, goal_definition_ptr);

  params->SetListFloat("ObserverModelParametric::EgoStateDeviationDist::Mean", {0.7, 1.8});
  params->SetListListFloat("ObserverModelParametric::EgoStateDeviationDist::Covariance",
                             {{0.2, 0.0}, 
                              {0.0, 0.2}});
  
  params->SetListFloat("ObserverModelParametric::OtherStateDeviationDist::Mean", {1.2, 3.5});
  params->SetListListFloat("ObserverModelParametric::OtherStateDeviationDist::Covariance",
                             {{0.3, 0.0}, 
                              {0.0, 2.1}});

  auto observer_parametric = std::make_shared<ObserverModelParametric>(params);
  world->SetObserverModel(observer_parametric);

  const auto agent1 = world->GetAgents().begin()->second;
  const auto agent2 = std::next(world->GetAgents().begin())->second;
  const auto agent3 = std::next(world->GetAgents().begin(), 2)->second;

  unsigned int num_steps = 10;
  const double step_time = 0.2;

  world->Step(step_time);

  auto get_sensed_state = [](const WorldPtr& world, const AgentId& agent_id) {
    return world->GetAgents().at(agent_id)->GetSensedWorld()->GetAgents().at(agent_id)->GetCurrentState();
  };

  auto calc_desired_velocity = [](const State& current_state, const State& prev_state, const double& step_time) {
    auto vlon = (current_state[StateDefinition::X_POSITION] - prev_state[StateDefinition::X_POSITION]) / step_time;
    auto vlat = (current_state[StateDefinition::Y_POSITION] - prev_state[StateDefinition::Y_POSITION]) / step_time;
    return sqrt(vlon*vlon + vlat*vlat);
  };

  for (int i = 0; i <= num_steps; ++i) {
    const auto prev_sense_state1 = get_sensed_state(world, agent1->GetAgentId());
    const auto prev_sense_state2 = get_sensed_state(world, agent2->GetAgentId());
    const auto prev_sense_state3 = get_sensed_state(world, agent3->GetAgentId());

    world->Step(step_time);

    const auto cur_sense_state1 = get_sensed_state(world, agent1->GetAgentId());
    const auto cur_sense_state2 = get_sensed_state(world, agent2->GetAgentId());
    const auto cur_sense_state3 = get_sensed_state(world, agent3->GetAgentId());

    // Frenet state is aligned along coordinate system, so no transformation required
    auto des_velocity1 = calc_desired_velocity(cur_sense_state1, prev_sense_state1, step_time);
    EXPECT_NEAR(des_velocity1, cur_sense_state1[StateDefinition::VEL_POSITION], 0.00001);

    auto des_velocity2 = calc_desired_velocity(cur_sense_state2, prev_sense_state2, step_time);
    EXPECT_NEAR(des_velocity2, cur_sense_state2[StateDefinition::VEL_POSITION], 0.00001);

    auto des_velocity3 = calc_desired_velocity(cur_sense_state3, prev_sense_state3, step_time);
    EXPECT_NEAR(des_velocity3, cur_sense_state3[StateDefinition::VEL_POSITION], 0.00001);
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
