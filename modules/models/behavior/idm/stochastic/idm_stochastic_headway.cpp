// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/models/behavior/idm/stochastic/idm_stochastic_headway.hpp"


namespace modules {
namespace models {
namespace behavior {


BehaviorIDMStochasticHeadway::BehaviorIDMStochasticHeadway(const commons::ParamsPtr& params) : BehaviorIDMClassic(params),
    param_dist_headway_(params->GetDistribution("BehaviorIDMStochasticHeadway::HeadwayDistribution",
     "From what distribution is the desired time headway sampled in each planning steo", "UniformDistribution1D")) {}


void BehaviorIDMStochasticHeadway::SampleParameters() {
  param_desired_time_head_way_ = param_dist_headway_.Sample()[0];
}

Trajectory BehaviorIDMClassic::Plan(float delta_time, const ObservedWorld& observed_world)
  SampleParameters();
  return BehaviorIDMClassic::Plan(delta_time, observed_world);
}

}  // namespace behavior
}  // namespace models
}  // namespace modules
