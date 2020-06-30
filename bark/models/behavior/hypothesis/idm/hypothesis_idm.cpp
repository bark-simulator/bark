// Copyright (c) 2020 Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/models/behavior/hypothesis/idm/hypothesis_idm.hpp"
#include "bark/world/observed_world.hpp"

namespace bark {
namespace models {
namespace behavior {

using bark::commons::transformation::FrenetPosition;
using bark::geometry::Point2d;
using bark::models::dynamic::State;
using bark::models::dynamic::StateDefinition;
using bark::world::objects::Agent;
using bark::world::objects::AgentId;
using bark::world::objects::AgentPtr;
using bark::world::FrontRearAgents;
using bark::commons::UniformDistribution1D;
using bark::models::behavior::BehaviorIDMStochastic;

BehaviorHypothesisIDM::BehaviorHypothesisIDM(const commons::ParamsPtr &params)
    : BehaviorIDMStochastic(params), BehaviorHypothesis(params),
      BehaviorModel(params),
      num_samples_(params->GetInt(
          "BehaviorHypothesisIDM::NumSamples",
          "Number of samples used for probability approximation", 10000)),
      num_buckets_(params->GetInt(
          "BehaviorHypothesisIDM::NumBuckets",
          "Number of buckets to split uniform probability space", 100)),
      buckets_lower_bound_(
          params->GetReal("BehaviorHypothesisIDM::BucketsLowerBound",
                          "Lower bounds where buckets start", -10.0f)),
      buckets_upper_bound_(
          params->GetReal("BehaviorHypothesisIDM::BucketsUpperBound",
                          "Upper bounds where buckets end", 10.0f)) {}

bark::commons::Probability BehaviorHypothesisIDM::GetProbability(
    const Action &action, const world::ObservedWorld &observed_world,
    const AgentId &agent_id) const {
  // First check if action fits to the hypothesis action type
  if (action.type() != typeid(Continuous1DAction) &&
      action.type() != typeid(LonLatAction)) {
    LOG(WARNING) << "Passed action's type not fitting to hypothesis.";
    return 0.0f;
  }

  // Get environment input variables to IDM model
  auto agent_ptr = observed_world.GetAgent(agent_id);
  if (!agent_ptr) {
    LOG(WARNING) << "Agent " << agent_id
                 << " does not exist. Returning zero prob.";
    return 0.0f;
  }
  Point2d agent_pos = agent_ptr->GetCurrentPosition();
  const auto &road_corridor = agent_ptr->GetRoadCorridor();
  if (!road_corridor) {
    LOG(ERROR) << "No road corridor found.";
    return 0.0f;
  }
  const auto &lane_corridor = road_corridor->GetCurrentLaneCorridor(agent_pos);
  if (!lane_corridor) {
    LOG(ERROR) << "No lane corridor found.";
    return 0.0f;
  }

  const auto others_observed_world =
      *observed_world.ObserveForOtherAgent(agent_id);
  IDMRelativeValues rel_values =
      CalcRelativeValues(others_observed_world, lane_corridor);

  // sample a lot of actions for this environment state to approximate the
  // probability
  double bucket_size =
      (buckets_upper_bound_ - buckets_lower_bound_) / num_buckets_;
  std::vector<unsigned int> sample_container(num_buckets_, 0);
  auto behavior_idm_stoch = std::dynamic_pointer_cast<BehaviorIDMStochastic>(
      BehaviorIDMStochastic::Clone());
  for (size_t i = 0; i < num_samples_; ++i) {
    behavior_idm_stoch->SampleParameters();
    const auto action_dist_sample = behavior_idm_stoch->GetTotalAcc(
        others_observed_world, rel_values, rel_values.leading_distance, 0.0f);
    const auto action_sample = action_dist_sample.first;
    if (action_sample < buckets_lower_bound_ ||
        action_sample > buckets_upper_bound_) {
      LOG(FATAL) << "Wrong bucket sizes, action=" << action_sample
                 << ", bucket_lower=" << buckets_lower_bound_
                 << ", bucket_upper=" << buckets_upper_bound_;
    }
    sample_container[std::floor((action_sample - buckets_lower_bound_) /
                                bucket_size)] += 1;
  }

  VLOG(4) << "Hypothesis action distribution of IDMStochasticHeadway with "
             "params \n "
          << GetParams()->AddChild("BehaviorIDMStochasticHeadway")->Print()
          << "\n"
          << sample_container;

  double idm_acc = 0.0f;
  if (action.type() == typeid(Continuous1DAction)) {
    idm_acc = boost::get<Continuous1DAction>(action);
  } else if (action.type() == typeid(LonLatAction)) {
    idm_acc = boost::get<LonLatAction>(action).acc_lon;
  }

  if (idm_acc > buckets_upper_bound_ || idm_acc < buckets_lower_bound_) {
    LOG(WARNING) << "Bucket bounds too small to include action " << idm_acc
                 << ". Returning zero prob.";
    return 0.0f;
  }

  // percentage in bucket the action falls into must be scaled by bucket size to
  // be independent
  // of number of buckets
  const auto &sample_count = sample_container[std::floor(
      (idm_acc - buckets_lower_bound_) / bucket_size)];
  // size of buckets is maximum density resolution or can be considered as
  // integration delta for density normalization
  return static_cast<double>(sample_count) / static_cast<double>(num_samples_);
}

} // namespace behavior
} // namespace models
} // namespace bark
