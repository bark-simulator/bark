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
                        previous_observed_world, delta_time);
  for (auto& agent : observed_world.GetOtherAgents()) {
    AddStateDeviationFrenet(agent.second, others_state_deviation_dist_,
                            previous_observed_world, delta_time);
  }

  return observed_world;
}

void ObserverModelParametric::AddStateDeviationFrenet(const AgentPtr& agent, const DistributionPtr& multi_dim_distribution,
                         const ObservedWorldPtr& previous_observed_world, const double& delta_time) const {
  // Get Current Frenet State of Agent
  const Point2d pos = agent->GetCurrentPosition();
  const auto& lane_corridor = agent->GetRoadCorridor()->GetCurrentLaneCorridor(pos);
  BARK_EXPECT_TRUE(bool(lane_corridor));
  FrenetState current_frenet_state(agent->GetCurrentState(), lane_corridor->GetCenterLine());

  // Add sampled frenet deviation to current frenet state
  const auto frenet_deviation = multi_dim_distribution->Sample();
  BARK_EXPECT_TRUE(frenet_deviation.size() == 2); // Lat, Long position deviation

  current_frenet_state.lon += frenet_deviation[0];
  current_frenet_state.lat += frenet_deviation[1];

  // Use previous observed state to calculate velocity deviation based on current lane corridor
  if(previous_observed_world) {
    FrenetState previous_frenet_state(
          previous_observed_world->GetAgents().at(agent->GetAgentId())->GetCurrentState(), lane_corridor->GetCenterLine());
    current_frenet_state.vlon = (previous_frenet_state.lon - current_frenet_state.lon)/delta_time;
    current_frenet_state.vlat = (previous_frenet_state.lat - current_frenet_state.lat)/delta_time;
  } // else: skip this step in the first time step, giving full observable velocity in this case
  
  // Convert back (calculates angle based on frenet velocities) and set dynamic agent state 
  const auto deviated_state = FrenetStateToDynamicState(current_frenet_state, lane_corridor->GetCenterLine());
  agent->SetCurrentState(deviated_state);

  return;
}

}  // namespace observer
}  // namespace models
}  // namespace bark
