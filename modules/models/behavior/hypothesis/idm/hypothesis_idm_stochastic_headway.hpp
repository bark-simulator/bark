// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_MODELS_BEHAVIOR_HYPOTHESIS_HYPOTHESIS_IDM_STOCHASTIC_HEADWAY_HPP_
#define MODULES_MODELS_BEHAVIOR_HYPOTHESIS_HYPOTHESIS_IDM_STOCHASTIC_HEADWAY_HPP_

#include "modules/models/behavior/idm/stochastic/idm_stochastic_headway.hpp"
#include "modules/models/behavior/hypothesis/behavior_hypothesis.hpp"

namespace modules {
namespace world {
namespace objects {
class Agent;
typedef std::shared_ptr<Agent> AgentPtr;
typedef unsigned int AgentId;
}  // namespace objects
class ObservedWorld;
}  // namespace world
namespace models {
namespace behavior {


class BehaviorHypothesisIDMStochasticHeadway : public BehaviorIDMStochasticHeadway, BehaviorHypothesis {
  public:
  explicit BehaviorIDMStochasticHeadway(const commons::ParamsPtr& params);

  virtual modules::commons::Probability GetProbability(const Action& action,
                             const world::ObservedWorld& observed_world,
                             const AgentId& agent_id) const;

  private: 
    const unsigned int num_samples_;
    const unsigned int num_buckets_;
    const modules::commons::RandomVariate1D buckets_upper_bound_;
    const modules::commons::RandomVariate1D buckets_lower_bound_; 
};


}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_HYPOTHESIS_HYPOTHESIS_IDM_STOCHASTIC_HEADWAY_HPP_