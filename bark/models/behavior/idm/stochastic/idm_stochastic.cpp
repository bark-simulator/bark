// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/models/behavior/idm/stochastic/idm_stochastic.hpp"

namespace bark {
namespace models {
namespace behavior {

BehaviorIDMStochastic::BehaviorIDMStochastic(const commons::ParamsPtr& params)
    : BehaviorIDMClassic(params),
      BehaviorModel(params),
      param_dist_headway_(
          params->GetDistribution("BehaviorIDMStochastic::HeadwayDistribution",
                                  "From what distribution is the desired time "
                                  "headway sampled in each planning steo",
                                  "UniformDistribution1D")),
      param_dist_spacing_(params->GetDistribution(
          "BehaviorIDMStochastic::SpacingDistribution",
          "From what distribution is the spacing sampled in each planning steo",
          "UniformDistribution1D")),
      param_dist_max_acc_(params->GetDistribution(
          "BehaviorIDMStochastic::MaxAccDistribution",
          "From what distribution is max acc sampled in each planning steo",
          "UniformDistribution1D")),
      param_dist_desired_vel_(params->GetDistribution(
          "BehaviorIDMStochastic::DesiredVelDistribution",
          "From what distribution is the desired velocity sampled in each "
          "planning steo",
          "UniformDistribution1D")),
      param_dist_comft_braking_(params->GetDistribution(
          "BehaviorIDMStochastic::ComftBrakingDistribution",
          "From what distribution is the comfortable braking sampled in each "
          "planning steo",
          "UniformDistribution1D")),
      param_dist_coolness_factor_(params->GetDistribution(
          "BehaviorIDMStochastic::CoolnessFactorDistribution",
          "From what distribution is the comfortable braking sampled in each "
          "planning steo",
          "UniformDistribution1D")),
      use_intention_mechanism_(params->GetBool("BehaviorIDMStochastic::UseIntentionMechanism",
      "True if intention changes yielding or no yielding are applied, "
      "otherwise default intent no yield is used", false)),
      current_yield_intent_(YieldIntent::NOT_INITIALIZED),
      duration_until_intent_change_(),
      world_time_at_last_intent_change_(),
      param_yielding_duration_(params->GetDistribution(
          "BehaviorIDMStochastic::YieldingDurationDistribution",
          "From what distribution is duration of the next yielding intent sampled",
          "UniformDistribution1D")),
      param_no_yielding_duration_(params->GetDistribution(
          "BehaviorIDMStochastic::NoYieldingDurationDistribution",
          "From what distribution is duration of the next no yielding intent sampled",
          "UniformDistribution1D")) {}

void BehaviorIDMStochastic::SampleParameters() {
  param_desired_time_head_way_ = param_dist_headway_->Sample()[0];
  param_minimum_spacing_ = param_dist_spacing_->Sample()[0];
  param_max_acceleration_ = param_dist_max_acc_->Sample()[0];
  param_desired_velocity_ = param_dist_desired_vel_->Sample()[0];
  param_comfortable_braking_acceleration_ =
      param_dist_comft_braking_->Sample()[0];
  param_coolness_factor_ = param_dist_coolness_factor_->Sample()[0];
}

ParameterRegions BehaviorIDMStochastic::GetParameterRegions() const {
  ParameterRegions parameter_regions;
  parameter_regions["BehaviorIDMStochastic::DesiredTimeHeadway"] =
      param_dist_headway_->GetSupport()[0];
  parameter_regions["BehaviorIDMStochastic::MinimumSpacing"] =
      param_dist_spacing_->GetSupport()[0];
  parameter_regions["BehaviorIDMStochastic::MaxAcceleration"] =
      param_dist_max_acc_->GetSupport()[0];
  parameter_regions["BehaviorIDMStochastic::DesiredVelocity"] =
      param_dist_desired_vel_->GetSupport()[0];
  parameter_regions["BehaviorIDMStochastic::ComfortableBraking"] =
      param_dist_comft_braking_->GetSupport()[0];
  parameter_regions["BehaviorIDMStochastic::CoolnessFactor"] =
      param_dist_coolness_factor_->GetSupport()[0];
  return parameter_regions;
}

void BehaviorIDMStochastic::HandleIntentionChange(const double& world_time) {
  if(YieldIntent::NOT_INITIALIZED) {
    world_time_at_last_intent_change_ = world_time;
    current_yield_intent_ = YieldIntent::YIELD;
    duration_until_intent_change_ = param_yielding_duration_->Sample()[0];
    max_lat_difference_to_be_front_ = k_max_lat_diff_yield;
    return;
  } 

  if(world_time - world_time_at_last_intent_change_ >= duration_until_intent_change_) {
    if(current_yield_intent_ == YieldIntent::YIELD) {
      current_yield_intent_ = YieldIntent::NO_YIELD;
      max_lat_difference_to_be_front_ = k_max_lat_diff_no_yield;
      duration_until_intent_change_ = param_no_yielding_duration_->Sample()[0]; 
    } else if(current_yield_intent_ == YieldIntent::NO_YIELD) {
      current_yield_intent_ = YieldIntent::YIELD;
      max_lat_difference_to_be_front_ = k_max_lat_diff_yield;
      duration_until_intent_change_ = param_yielding_duration_->Sample()[0]; 
    }
    world_time_at_last_intent_change_ = world_time;
  }
}

void BehaviorIDMStochastic::ChangeSeed(const bark::commons::RandomSeed& new_seed) {
  param_dist_headway_->ChangeSeed(new_seed);
  param_dist_spacing_->ChangeSeed(new_seed);
  param_dist_max_acc_->ChangeSeed(new_seed);
  param_dist_desired_vel_->ChangeSeed(new_seed);
  param_dist_comft_braking_->ChangeSeed(new_seed);
  param_dist_coolness_factor_->ChangeSeed(new_seed);
}

Trajectory BehaviorIDMStochastic::Plan(double delta_time,
                                       const ObservedWorld& observed_world) {
  SampleParameters();
  if(use_intention_mechanism_) {
    HandleIntentionChange(observed_world.GetWorldTime());
  }
  return BehaviorIDMClassic::Plan(delta_time, observed_world);
}

}  // namespace behavior
}  // namespace models
}  // namespace bark
