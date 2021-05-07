// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include <memory>
#include "bark/models/observer/observer_model_parametric.hpp"
#include "bark/commons/distribution/distribution.hpp"
#include "bark/commons/transformation/frenet_state.hpp"
#include "bark/world/observed_world.hpp"

namespace bark {
namespace models {
namespace observer {
using namespace bark::world;
using namespace bark::world::objects;
using namespace bark::commons::transformation;
using namespace bark::commons;

ObserverModelParametric::ObserverModelParametric(bark::commons::ParamsPtr params) :
  ObserverModel(params),
  ego_state_deviation_dist_(params->GetDistribution("ObserverModelParametric::EgoStateDeviationDist",
                                  "From what distribution is the ego frenet state deviations "
                                  "sampled, must have dimension = 5",
                                  "MultivariateDistribution")),
  others_state_deviation_dist_(params->GetDistribution("ObserverModelParametric::OtherStateDeviationDist",
                                  "From what distribution is the others frenet state deviation"
                                  "sampled, must have dimension = 5",
                                  "MultivariateDistribution")) {}

ObserverModelParametric::ObserverModelParametric(const ObserverModelParametric& observer_model) :
      ObserverModel(observer_model.GetParams()),
      ego_state_deviation_dist_(observer_model.ego_state_deviation_dist_),
      others_state_deviation_dist_(observer_model.others_state_deviation_dist_) {}


ObservedWorld ObserverModelParametric::Observe(
    const WorldPtr& world, const AgentId& agent_id) {
  
  // Clone world here since otherwise we change global world state
  auto observed_world = ObservedWorld(world->Clone(), agent_id);

  const auto observe_only_for = GetObserveOnlyForAgents();
  if(!observe_only_for.empty() && std::find(observe_only_for.begin(),
     observe_only_for.end(), agent_id) == observe_only_for.end()) {
    return observed_world;
  }
  const auto previous_observed_world = world->GetAgents().at(agent_id)->GetSensedWorld();
  double delta_time = 0.0f; // must only be valid when previous observed world exists
  if(previous_observed_world) {
      delta_time = world->GetWorldTime() - previous_observed_world->GetWorldTime();
  }
  AddStateDeviationFrenet(observed_world.GetEgoAgent(), ego_state_deviation_dist_,
                        previous_observed_world);
  for (auto& agent : observed_world.GetOtherAgents()) {
    AddStateDeviationFrenet(agent.second, others_state_deviation_dist_,
                            previous_observed_world);
  }

  return observed_world;
}

void ObserverModelParametric::AddStateDeviationFrenet(const AgentPtr& agent, const DistributionPtr& multi_dim_distribution,
                         const ObservedWorldPtr& previous_observed_world) const {
  // Add sampled frenet deviation to current frenet state
  const auto state_deviation = multi_dim_distribution->Sample();
  BARK_EXPECT_TRUE(state_deviation.size() == 4); // directional, orthogonal position deviation + velocity and angular deviation

  // Get Current Frenet State of Agent
  auto state = agent->GetCurrentState();
  const auto theta = state[StateDefinition::THETA_POSITION];
  state[StateDefinition::X_POSITION] += state_deviation[0]*cos(theta) + state_deviation[1]*sin(theta);
  state[StateDefinition::Y_POSITION] += state_deviation[0]*sin(theta) + state_deviation[1]*cos(theta);
  state[StateDefinition::THETA_POSITION] += state_deviation[2];
  state[StateDefinition::VEL_POSITION] += state_deviation[3];

  agent->SetCurrentState(state);
}

}  // namespace observer
}  // namespace models
}  // namespace bark
