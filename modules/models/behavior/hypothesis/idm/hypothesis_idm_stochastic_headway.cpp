// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/models/behavior/hypothesis/idm/hypothesis_idm_stochastic_headway.hpp"
#include "modules/world/observed_world.hpp"

namespace modules {
namespace models {
namespace behavior {

using modules::commons::transformation::FrenetPosition;
using modules::geometry::Point2d;
using modules::models::dynamic::State;
using modules::models::dynamic::StateDefinition;
using modules::world::objects::Agent;
using modules::world::objects::AgentPtr;
using modules::world::FrontRearAgents;
using modules::commons::UniformDistribution1D;

BehaviorHypothesisIDMStochasticHeadway::BehaviorHypothesisIDMStochasticHeadway(const commons::ParamsPtr& params) :
                             BehaviorIDMStochasticHeadway(params),
                             BehaviorHypothesis(params),
                             BehaviorModel(params),
                             num_samples_(params->GetInt("BehaviorHypothesisIDMStochasticHeadway::NumSamples",
                                 "Number of samples used for probability approximation", 10000)),
                             num_buckets_(params->GetInt("BehaviorHypothesisIDMStochasticHeadway::NumBuckets",
                                 "Number of buckets to split uniform probability space", 100)),
                             buckets_lower_bound_(params->GetReal("BehaviorHypothesisIDMStochasticHeadway::BucketsLowerBound",
                                 "Lower bounds where buckets start", -10.0f)),
                             buckets_upper_bound_(params->GetReal("BehaviorHypothesisIDMStochasticHeadway::BucketsUpperBound",
                                 "Upper bounds where buckets end", 10.0f)) 
                            {}

modules::commons::Probability BehaviorHypothesisIDMStochasticHeadway::GetProbability(const Action& action,
                            const world::ObservedWorld& observed_world,
                            const AgentId& agent_id) const {
  // First check if action fits to the hypothesis action type
  if(action.type() != typeid(Continuous1DAction)) {
    LOG(WARNING) << "Passed action's type not fitting to hypothesis.";
    return 0.0f;
  }

   // Get environment input variables to IDM model
  auto agent_ptr = observed_world.GetAgent(agent_id);
  if(!agent_ptr) {
    LOG(WARNING) << "Agent " << agent_id << " does not exist. Returning zero prob.";
    return 0.0f;
  }
  Point2d agent_pos = agent_ptr->GetCurrentPosition();
  const auto& road_corridor = agent_ptr->GetRoadCorridor();
  if (!road_corridor) {
    LOG(ERROR) << "No road corridor found.";
    return 0.0f;
  }
  const auto& lane_corridor = road_corridor->GetCurrentLaneCorridor(agent_pos);
  if (!lane_corridor) {
    LOG(ERROR) << "No lane corridor found.";
    return 0.0f;
  }
  FrontRearAgents front_rear_agents = observed_world.GetAgentFrontRearForId(agent_id, lane_corridor);
  auto leading_vehicle = front_rear_agents.front;
  std::shared_ptr<const Agent> ego_agent = observed_world.GetAgent(agent_id);

  dynamic::State ego_vehicle_state = observed_world.CurrentEgoState();
  auto vel_ego = ego_vehicle_state(StateDefinition::VEL_POSITION);

  double net_distance = std::numeric_limits<double>::max();
  double vel_other = std::numeric_limits<double>::max();
  if(leading_vehicle.first) {
    net_distance = BehaviorIDMStochasticHeadway::CalcNetDistance(ego_agent, leading_vehicle.first);
    dynamic::State other_vehicle_state =
        leading_vehicle.first->GetCurrentState();
    vel_other = other_vehicle_state(StateDefinition::VEL_POSITION);
  }

  // sample a lot of actions for this environment state to approximate the probability
  double bucket_size = (buckets_upper_bound_ - buckets_lower_bound_)/num_buckets_;
  std::vector<std::vector<double>> sample_container(num_buckets_,
         std::vector<Continuous1DAction>(std::ceil(static_cast<float>(num_samples_)/num_buckets_)));
   for(size_t i = 0; i < num_samples_; ++i) {
    auto action_sample = BehaviorIDMStochasticHeadway::CalcIDMAcc(net_distance, vel_ego, vel_other);
    BARK_EXPECT_TRUE(action_sample >= buckets_lower_bound_);
    BARK_EXPECT_TRUE(action_sample <= buckets_upper_bound_);
    sample_container[std::floor((action_sample-buckets_lower_bound_)/bucket_size)].push_back(action_sample);
  }

  auto idm_acc = boost::get<Continuous1DAction>(action);
  if(idm_acc > buckets_upper_bound_ || idm_acc < buckets_lower_bound_) {
    LOG(WARNING) << "Bucket bounds too small to include action " << idm_acc << ". Returning zero prob.";
    return 0.0f;
  } 

  // percentage in bucket the action falls into must be scaled by bucket size to be independent
  // of number of buckets
  const auto& bucket = sample_container[std::floor((idm_acc-buckets_lower_bound_)/bucket_size)];
  return static_cast<double>(bucket.size())/(num_samples_*bucket_size);
}

}  // namespace behavior
}  // namespace models
}  // namespace modules
