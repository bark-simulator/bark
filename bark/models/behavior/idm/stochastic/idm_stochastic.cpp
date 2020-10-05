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
  parameter_regions["BehaviorIDMStochastic::DesiredTimeHeadway"] = param_dist_headway_->GetSupport()[0];
  parameter_regions["BehaviorIDMStochastic::MinimumSpacing"] = param_dist_spacing_->GetSupport()[0];
  parameter_regions["BehaviorIDMStochastic::MaxAcceleration"] = param_dist_max_acc_->GetSupport()[0];
  parameter_regions["BehaviorIDMStochastic::DesiredVelocity"] = param_dist_desired_vel_->GetSupport()[0];
  parameter_regions["BehaviorIDMStochastic::ComfortableBraking"] = param_dist_comft_braking_->GetSupport()[0];
  parameter_regions["BehaviorIDMStochastic::CoolnessFactor"] = param_dist_coolness_factor_->GetSupport()[0];
  return parameter_regions;
}

Trajectory BehaviorIDMStochastic::Plan(float delta_time,
                                       const ObservedWorld& observed_world) {
  SampleParameters();
  return BehaviorIDMClassic::Plan(delta_time, observed_world);
}

}  // namespace behavior
}  // namespace models
}  // namespace bark
