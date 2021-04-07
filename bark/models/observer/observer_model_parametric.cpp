// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include <string>
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
using world::renderer::RendererPtr;

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

  // NOTE: generate child renderer for the observed world
  RendererPtr renderer = world->GetRenderer()->AddRendererChild(
    std::to_string(agent_id));
  observed_world.SetRenderer(renderer);

  AddStateDeviationFrenet(observed_world.GetEgoAgent(), ego_state_deviation_dist_);
  for (auto& agent : observed_world.GetOtherAgents()) {
    AddStateDeviationFrenet(agent.second, others_state_deviation_dist_);
  }

  return observed_world;
}

void ObserverModelParametric::AddStateDeviationFrenet(const AgentPtr& agent, const DistributionPtr& multi_dim_distribution) const {
  // Get Current Frenet State of Agent
  const Point2d pos = agent->GetCurrentPosition();
  const auto& lane_corridor = agent->GetRoadCorridor()->GetCurrentLaneCorridor(pos);
  BARK_EXPECT_TRUE(bool(lane_corridor));
  FrenetState current_frenet_state(agent->GetCurrentState(), lane_corridor->GetCenterLine());

  // Add sampled frenet deviation to current frenet state
  const auto frenet_deviation = multi_dim_distribution->Sample();
  BARK_EXPECT_TRUE(frenet_deviation.size() == 5); // Lat, Long, vlat, vlon, Orientation

  current_frenet_state.lon += frenet_deviation[0];
  current_frenet_state.lat += frenet_deviation[1];
  current_frenet_state.vlat += frenet_deviation[2];
  current_frenet_state.vlon += frenet_deviation[3];
  current_frenet_state.angle += frenet_deviation[4];
  
  // Convert back and set dynamic agent state 
  const auto deviated_state = FrenetStateToDynamicState(current_frenet_state, lane_corridor->GetCenterLine());
  agent->SetCurrentState(deviated_state);

  return;
}

}  // namespace observer
}  // namespace models
}  // namespace bark
